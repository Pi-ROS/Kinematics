#include "controllers.hpp"
#include "ros.hpp"

void enforceVelocityLimits(VEC3 &v, double max);

double vector_field_controller::scalarVelocity(VEC3 e, int iter) {
    if (iter < vector_field_controller::N0)
        return e.norm() * (iter / vector_field_controller::N0);
    else
        return e.norm();
}

double vector_field_controller::scalarRotVelocity(VEC3 e, int iter) {
    if (iter < vector_field_controller::N0)
        return e.norm() * (iter / vector_field_controller::N0);
    else
        return e.norm();
}

VEC3 vector_field_controller::vectorField(VEC3 p_curr, VEC3 p_f) {
    double radius = sqrt(pow(p_curr(0), 2) + pow(p_curr(1), 2));
    VEC3 vecField;
    if (radius > vector_field_controller::r0) {
        vecField << 0, 0, 0;
    }
    else {
        double th = atan2(p_curr(1), p_curr(0));
        if (th < 0)
            th += 2 * M_PI;
        double sign = 1 ? p_f(0) > p_curr(0) : -1;
        th += sign * M_PI / 2;
        vecField(0) = cos(th);
        vecField(1) = sin(th);
        vecField(2) = 0;
        vecField *= 4 - 2*radius/vector_field_controller::r0;
    }
    return vecField;
}

VEC3 vector_field_controller::velocity(VEC3 e, VEC3 p_curr, VEC3 p_f, int iter) {
    double v = scalarVelocity(e, iter);
    double e_norm = e.norm();
    VEC3 potentialField = vectorField(p_curr, p_f);
    double crossProduct_z = potentialField(0)*e(1) - potentialField(1)*e(0);
    if (crossProduct_z < 0 && p_f(0) > p_curr(0) || crossProduct_z > 0 && p_f(0) < p_curr(0))
        potentialField << 0, 0, 0;
    VEC3 numerator = e / e_norm + potentialField;
    double denominator = numerator.norm();

    VEC3 vel = vector_field_controller::Lambda * v * numerator / denominator;
    enforceVelocityLimits(vel, vector_field_controller::MaxVel);
    return vel;
}

VEC3 vector_field_controller::rotationalVelocity(VEC3 e, int iter) {
    double w = scalarRotVelocity(e, iter);
    double e_norm = e.norm();
    VEC3 vel = vector_field_controller::Lambda * w * e / e_norm;
    enforceVelocityLimits(vel, vector_field_controller::MaxRotVel);
    return vel;
}

VEC6 vector_field_controller::qDot(Robot &r, VEC6 q, VEC6 e, VEC3 p_f, int iter) {
    MAT6 Jac = r.jacobian(q);
    VEC3 e_tau;
    VEC3 e_ro;
    e_tau << e(0), e(1), e(2);
    e_ro << e(3), e(4), e(5);

    SE3 T_curr = r.forwardKinematics(q);
    VEC3 p_curr = SE3Operations::tau(T_curr);
    VEC3 v_tau = velocity(e_tau, p_curr, p_f, iter);
    VEC3 v_ro = rotationalVelocity(e_ro, iter);
    VEC6 v;
    v << v_tau(0), v_tau(1), v_tau(2), v_ro(0), v_ro(1), v_ro(2);

    MAT6 Ak = LM::Ak(Jac, e.norm());
    VEC6 gk = LM::gk(Jac, v);
    VEC6 qdot = Ak.inverse() * gk;
    return qdot;
}

void vector_field_controller::vectorFieldController(Robot &r, SE3 &T_des) {
    ros::Rate loop_rate(1/DT);
    int i = 0;
    VEC6 qk = r.joints.q();
    VEC3 q_gripper = r.joints.q_gripper();

    VEC3 p_f = SE3Operations::tau(T_des);
    while (true) {
        i++;
        SE3 T_curr = r.forwardKinematics(qk);
        VEC6 e = LM::error(T_curr, T_des);
        if (e.norm() < vector_field_controller::ErrThresh) break;
        VEC6 qdot = vector_field_controller::qDot(r, qk, e, p_f, i);
        qk += qdot * DT;
        publishJoints(pub_jstate, qk, q_gripper);
        loop_rate.sleep();
    }

    r.joints.update();
}

void redundant_controller::redundantController(Robot &r, VEC3 &x_f){
    ros::Rate loop_rate(1/DT);
    VEC6 q_k = r.joints.q();
    VEC3 q_gripper = r.joints.q_gripper();

    VEC3 x_0 = SE3Operations::tau(r.forwardKinematics(q_k));
    VEC3 x_e = x_0; // x_e: current position
    VEC3 x_des; // x_des: desired (next) position
    VEC3 v_des = (x_f - x_0) / redundant_controller::T;    
       
    int N = redundant_controller::T / DT;
    for(int i = 1; i <= N; i++){
        x_des = x_e + v_des * DT;
        MAT6 jac = r.jacobian(q_k);
        VEC6 qdot = redundant_controller::computeQdot(jac, q_k, x_e, x_des, v_des);
        q_k = q_k + qdot * DT;
        publishJoints(pub_jstate, q_k, q_gripper);
       
        loop_rate.sleep();
        x_e = x_des;
    }
    r.joints.update(q_k);
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat) // choose appropriately
{   
    typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4};
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}


VEC6 redundant_controller::computeQ0dot(VEC6 q){
    VEC6 result;
    double D1 = 1 / (redundant_controller::q1max - redundant_controller::q1min);
    double D2 = 1 / (redundant_controller::q2max - redundant_controller::q2min);
    double D3 = 1 / (redundant_controller::q3max - redundant_controller::q3min);
    double D4 = 1 / (redundant_controller::q4max - redundant_controller::q4min);
    double D5 = 1 / (redundant_controller::q5max - redundant_controller::q5min);
    double D6 = 1 / (redundant_controller::q6max - redundant_controller::q6min);

    double q1t = q(0) - redundant_controller::q1avg;
    double q2t = q(1) - redundant_controller::q2avg;
    double q3t = q(2) - redundant_controller::q3avg;
    double q4t = q(3) - redundant_controller::q4avg;
    double q5t = q(4) - redundant_controller::q5avg;
    double q6t = q(5) - redundant_controller::q6avg;

    result << D1 * D1 * q1t, D2 * D2 * q2t, D3 * D3 * q3t, D4 * D4 * q4t, D5 * D5 * q5t, D6 * D6 * q6t;
    result = result / 6.0;
    return result;
}

VEC6 redundant_controller::computeQdot(MAT6 &Jac, VEC6 q, VEC3 xe, VEC3 xd, VEC3 vd){
    MAT36 Jtras;
    // translation
    Jtras << Jac(0,0), Jac(0,1), Jac(0,2), Jac(0,3), Jac(0,4), Jac(0,5),
             Jac(1,0), Jac(1,1), Jac(1,2), Jac(1,3), Jac(1,4), Jac(1,5),
             Jac(2,0), Jac(2,1), Jac(2,2), Jac(2,3), Jac(2,4), Jac(2,5);

    VEC6 q0dot = computeQ0dot(q);
    Eigen::MatrixXd JtransInv = pseudoinverse(Jtras);
    VEC6 qdot = JtransInv * vd + (Eigen::Matrix<double, 6, 6>::Identity() - JtransInv * Jtras) * q0dot;
    return qdot;
}

void velocityController(Robot &r, double dt, double v_des, VEC6 q_f, bool ascent)
{   
    double v_ref = 0.0;
    VEC6 q_k = r.joints.q();
    VEC3 q_gripper = r.joints.q_gripper();

    VEC6 e;
    double e_norm;
    ros::Rate rate(1 / dt);
    
    VEC6 q_des;
    // The last joint will be updated only at the end of the ascent
    if (ascent) {
        q_des << q_f(0), q_f(1), q_f(2), q_f(3), q_f(4), q_k(5);
    } else q_des = q_f;

    while (true)
    {
        e = q_des - q_k;
        e_norm = e.norm();
        if (e_norm != 0.0)
        {
            v_ref += 0.005 * (v_des - v_ref);
            q_k += dt * v_ref * e / e_norm;
            
            publishJoints(pub_jstate, q_k, q_gripper);
        }
        rate.sleep();
        if (e_norm < 0.001)
        {
           
            // The final joint configuration is published
            publishJoints(pub_jstate, q_f, q_gripper);
            break;
        }
    }
    r.joints.update();
}

void enforceVelocityLimits(VEC3 &v, double max) {
    for (int i=0; i<3; ++i)
        if (v(i) > max)
            v(i) = max;
}

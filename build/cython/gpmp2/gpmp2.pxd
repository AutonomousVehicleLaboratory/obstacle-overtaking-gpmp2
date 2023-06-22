from gtsam.gtsam cimport *
from gtsam_eigency.core cimport *
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from libcpp.set cimport set
from libcpp.map cimport map
from libcpp cimport bool

cdef extern from "boost/shared_ptr.hpp" namespace "boost":
    cppclass shared_ptr[T]:
        shared_ptr()
        shared_ptr(T*)
        T* get()
        long use_count() const
        T& operator*()

    cdef shared_ptr[T] dynamic_pointer_cast[T,U](const shared_ptr[U]& r)
    cdef shared_ptr[T] make_shared[T](const T& r)

cdef extern from "gpmp2/geometry/Pose2Vector.h" namespace "gpmp2":
    cdef cppclass CPose2Vector "gpmp2::Pose2Vector":
        CPose2Vector() except +
        CPose2Vector(const CPose2& pose, const VectorXd& c) except +

        VectorXd configuration() except +
        CPose2 pose() except +
        void print_ "print"(string s) except +

cdef class Pose2Vector:
    cdef shared_ptr[CPose2Vector] CPose2Vector_
    @staticmethod
    cdef Pose2Vector cyCreateFromShared(const shared_ptr[CPose2Vector]& other)


cdef extern from "gpmp2/gp/GaussianProcessPriorLinear.h" namespace "gpmp2":
    cdef cppclass CGaussianProcessPriorLinear "gpmp2::GaussianProcessPriorLinear"(CNoiseModelFactor):
        CGaussianProcessPriorLinear(size_t key1, size_t key2, size_t key3, size_t key4, double delta, const shared_ptr[CnoiseModel_Base]& Qc_model) except +

        VectorXd evaluateError(const VectorXd& pose1, const VectorXd& vel1, const VectorXd& pose2, const VectorXd& vel2) except +

cdef class GaussianProcessPriorLinear(NoiseModelFactor):
    cdef shared_ptr[CGaussianProcessPriorLinear] CGaussianProcessPriorLinear_
    @staticmethod
    cdef GaussianProcessPriorLinear cyCreateFromShared(const shared_ptr[CGaussianProcessPriorLinear]& other)


cdef extern from "gpmp2/gp/GaussianProcessPriorPose2.h" namespace "gpmp2":
    cdef cppclass CGaussianProcessPriorPose2 "gpmp2::GaussianProcessPriorPose2"(CNoiseModelFactor):
        CGaussianProcessPriorPose2(size_t key1, size_t key2, size_t key3, size_t key4, double delta, const shared_ptr[CnoiseModel_Base]& Qc_model) except +


cdef class GaussianProcessPriorPose2(NoiseModelFactor):
    cdef shared_ptr[CGaussianProcessPriorPose2] CGaussianProcessPriorPose2_
    @staticmethod
    cdef GaussianProcessPriorPose2 cyCreateFromShared(const shared_ptr[CGaussianProcessPriorPose2]& other)


cdef extern from "gpmp2/gp/GaussianProcessPriorPose2Vector.h" namespace "gpmp2":
    cdef cppclass CGaussianProcessPriorPose2Vector "gpmp2::GaussianProcessPriorPose2Vector"(CNoiseModelFactor):
        CGaussianProcessPriorPose2Vector(size_t key1, size_t key2, size_t key3, size_t key4, double delta, const shared_ptr[CnoiseModel_Base]& Qc_model) except +


cdef class GaussianProcessPriorPose2Vector(NoiseModelFactor):
    cdef shared_ptr[CGaussianProcessPriorPose2Vector] CGaussianProcessPriorPose2Vector_
    @staticmethod
    cdef GaussianProcessPriorPose2Vector cyCreateFromShared(const shared_ptr[CGaussianProcessPriorPose2Vector]& other)


cdef extern from "gpmp2/gp/GaussianProcessInterpolatorLinear.h" namespace "gpmp2":
    cdef cppclass CGaussianProcessInterpolatorLinear "gpmp2::GaussianProcessInterpolatorLinear":
        CGaussianProcessInterpolatorLinear(const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +

        VectorXd interpolatePose(const VectorXd& pose1, const VectorXd& vel1, const VectorXd& pose2, const VectorXd& vel2) except +
        VectorXd interpolateVelocity(const VectorXd& pose1, const VectorXd& vel1, const VectorXd& pose2, const VectorXd& vel2) except +

cdef class GaussianProcessInterpolatorLinear:
    cdef shared_ptr[CGaussianProcessInterpolatorLinear] CGaussianProcessInterpolatorLinear_
    @staticmethod
    cdef GaussianProcessInterpolatorLinear cyCreateFromShared(const shared_ptr[CGaussianProcessInterpolatorLinear]& other)


cdef extern from "gpmp2/kinematics/Arm.h" namespace "gpmp2":
    cdef cppclass CArm "gpmp2::Arm":
        CArm(size_t dof, const VectorXd& a, const VectorXd& alpha, const VectorXd& d) except +
        CArm(size_t dof, const VectorXd& a, const VectorXd& alpha, const VectorXd& d, const CPose3& base_pose) except +
        CArm(size_t dof, const VectorXd& a, const VectorXd& alpha, const VectorXd& d, const CPose3& base_pose, const VectorXd& theta_bias) except +

        VectorXd a() except +
        VectorXd alpha() except +
        CPose3 base_pose() except +
        VectorXd d() except +
        size_t dof() except +
        MatrixXd forwardKinematicsPose(const VectorXd& jp) except +
        MatrixXd forwardKinematicsPosition(const VectorXd& jp) except +
        MatrixXd forwardKinematicsVel(const VectorXd& jp, const VectorXd& jv) except +

cdef class Arm:
    cdef shared_ptr[CArm] CArm_
    @staticmethod
    cdef Arm cyCreateFromShared(const shared_ptr[CArm]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileBase.h" namespace "gpmp2":
    cdef cppclass CPose2MobileBase "gpmp2::Pose2MobileBase":
        CPose2MobileBase() except +

        size_t dof() except +
        MatrixXd forwardKinematicsPose(const CPose2& jp) except +
        MatrixXd forwardKinematicsPosition(const CPose2& jp) except +
        MatrixXd forwardKinematicsVel(const CPose2& jp, const VectorXd& jv) except +
        size_t nr_links() except +

cdef class Pose2MobileBase:
    cdef shared_ptr[CPose2MobileBase] CPose2MobileBase_
    @staticmethod
    cdef Pose2MobileBase cyCreateFromShared(const shared_ptr[CPose2MobileBase]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileArm.h" namespace "gpmp2":
    cdef cppclass CPose2MobileArm "gpmp2::Pose2MobileArm":
        CPose2MobileArm(const CArm& arm) except +
        CPose2MobileArm(const CArm& arm, const CPose3& base_T_arm) except +

        CArm arm() except +
        CPose3 base_T_arm() except +
        size_t dof() except +
        MatrixXd forwardKinematicsPose(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsPosition(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsVel(const CPose2Vector& jp, const VectorXd& jv) except +
        size_t nr_links() except +

cdef class Pose2MobileArm:
    cdef shared_ptr[CPose2MobileArm] CPose2MobileArm_
    @staticmethod
    cdef Pose2MobileArm cyCreateFromShared(const shared_ptr[CPose2MobileArm]& other)


cdef extern from "gpmp2/kinematics/Pose2Mobile2Arms.h" namespace "gpmp2":
    cdef cppclass CPose2Mobile2Arms "gpmp2::Pose2Mobile2Arms":
        CPose2Mobile2Arms(const CArm& arm1, const CArm& arm2) except +
        CPose2Mobile2Arms(const CArm& arm1, const CArm& arm2, const CPose3& base_T_arm1, const CPose3& base_T_arm2) except +

        CArm arm1() except +
        CArm arm2() except +
        CPose3 base_T_arm1() except +
        CPose3 base_T_arm2() except +
        size_t dof() except +
        MatrixXd forwardKinematicsPose(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsPosition(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsVel(const CPose2Vector& jp, const VectorXd& jv) except +
        size_t nr_links() except +

cdef class Pose2Mobile2Arms:
    cdef shared_ptr[CPose2Mobile2Arms] CPose2Mobile2Arms_
    @staticmethod
    cdef Pose2Mobile2Arms cyCreateFromShared(const shared_ptr[CPose2Mobile2Arms]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileVetLinArm.h" namespace "gpmp2":
    cdef cppclass CPose2MobileVetLinArm "gpmp2::Pose2MobileVetLinArm":
        CPose2MobileVetLinArm(const CArm& arm) except +
        CPose2MobileVetLinArm(const CArm& arm, const CPose3& base_T_torso, const CPose3& torso_T_arm, bool reverse_linact) except +

        CArm arm() except +
        CPose3 base_T_torso() except +
        size_t dof() except +
        MatrixXd forwardKinematicsPose(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsPosition(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsVel(const CPose2Vector& jp, const VectorXd& jv) except +
        size_t nr_links() except +
        bool reverse_linact() except +
        CPose3 torso_T_arm() except +

cdef class Pose2MobileVetLinArm:
    cdef shared_ptr[CPose2MobileVetLinArm] CPose2MobileVetLinArm_
    @staticmethod
    cdef Pose2MobileVetLinArm cyCreateFromShared(const shared_ptr[CPose2MobileVetLinArm]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileVetLin2Arms.h" namespace "gpmp2":
    cdef cppclass CPose2MobileVetLin2Arms "gpmp2::Pose2MobileVetLin2Arms":
        CPose2MobileVetLin2Arms(const CArm& arm1, const CArm& arm2) except +
        CPose2MobileVetLin2Arms(const CArm& arm1, const CArm& arm2, const CPose3& base_T_torso, const CPose3& torso_T_arm1, const CPose3& torso_T_arm2, bool reverse_linact) except +

        CArm arm1() except +
        CArm arm2() except +
        CPose3 base_T_torso() except +
        size_t dof() except +
        MatrixXd forwardKinematicsPose(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsPosition(const CPose2Vector& jp) except +
        MatrixXd forwardKinematicsVel(const CPose2Vector& jp, const VectorXd& jv) except +
        size_t nr_links() except +
        bool reverse_linact() except +
        CPose3 torso_T_arm1() except +
        CPose3 torso_T_arm2() except +

cdef class Pose2MobileVetLin2Arms:
    cdef shared_ptr[CPose2MobileVetLin2Arms] CPose2MobileVetLin2Arms_
    @staticmethod
    cdef Pose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CPose2MobileVetLin2Arms]& other)


cdef extern from "gpmp2/kinematics/PointRobot.h" namespace "gpmp2":
    cdef cppclass CPointRobot "gpmp2::PointRobot":
        CPointRobot(size_t dof, size_t nr_links) except +

        size_t dof() except +
        MatrixXd forwardKinematicsPose(const VectorXd& jp) except +
        MatrixXd forwardKinematicsPosition(const VectorXd& jp) except +
        MatrixXd forwardKinematicsVel(const VectorXd& jp, const VectorXd& jv) except +
        size_t nr_links() except +

cdef class PointRobot:
    cdef shared_ptr[CPointRobot] CPointRobot_
    @staticmethod
    cdef PointRobot cyCreateFromShared(const shared_ptr[CPointRobot]& other)


cdef extern from "gpmp2/kinematics/RobotModel.h" namespace "gpmp2":
    cdef cppclass CBodySphere "gpmp2::BodySphere":
        CBodySphere(size_t id, double r, const CPoint3& c) except +


cdef class BodySphere:
    cdef shared_ptr[CBodySphere] CBodySphere_
    @staticmethod
    cdef BodySphere cyCreateFromShared(const shared_ptr[CBodySphere]& other)


cdef extern from "gpmp2/kinematics/RobotModel.h" namespace "gpmp2":
    cdef cppclass CBodySphereVector "gpmp2::BodySphereVector":
        CBodySphereVector() except +

        void push_back(const CBodySphere& sphere) except +

cdef class BodySphereVector:
    cdef shared_ptr[CBodySphereVector] CBodySphereVector_
    @staticmethod
    cdef BodySphereVector cyCreateFromShared(const shared_ptr[CBodySphereVector]& other)


cdef extern from "gpmp2/kinematics/ArmModel.h" namespace "gpmp2":
    cdef cppclass CArmModel "gpmp2::ArmModel":
        CArmModel(const CArm& arm, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CArm fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const VectorXd& conf) except +
        double sphere_radius(size_t i) except +

cdef class ArmModel:
    cdef shared_ptr[CArmModel] CArmModel_
    @staticmethod
    cdef ArmModel cyCreateFromShared(const shared_ptr[CArmModel]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileBaseModel.h" namespace "gpmp2":
    cdef cppclass CPose2MobileBaseModel "gpmp2::Pose2MobileBaseModel":
        CPose2MobileBaseModel(const CPose2MobileBase& r, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPose2MobileBase fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const CPose2& conf) except +
        double sphere_radius(size_t i) except +

cdef class Pose2MobileBaseModel:
    cdef shared_ptr[CPose2MobileBaseModel] CPose2MobileBaseModel_
    @staticmethod
    cdef Pose2MobileBaseModel cyCreateFromShared(const shared_ptr[CPose2MobileBaseModel]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileArmModel.h" namespace "gpmp2":
    cdef cppclass CPose2MobileArmModel "gpmp2::Pose2MobileArmModel":
        CPose2MobileArmModel(const CPose2MobileArm& r, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPose2MobileArm fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const CPose2Vector& conf) except +
        double sphere_radius(size_t i) except +

cdef class Pose2MobileArmModel:
    cdef shared_ptr[CPose2MobileArmModel] CPose2MobileArmModel_
    @staticmethod
    cdef Pose2MobileArmModel cyCreateFromShared(const shared_ptr[CPose2MobileArmModel]& other)


cdef extern from "gpmp2/kinematics/Pose2Mobile2ArmsModel.h" namespace "gpmp2":
    cdef cppclass CPose2Mobile2ArmsModel "gpmp2::Pose2Mobile2ArmsModel":
        CPose2Mobile2ArmsModel(const CPose2Mobile2Arms& r, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPose2Mobile2Arms fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const CPose2Vector& conf) except +
        double sphere_radius(size_t i) except +

cdef class Pose2Mobile2ArmsModel:
    cdef shared_ptr[CPose2Mobile2ArmsModel] CPose2Mobile2ArmsModel_
    @staticmethod
    cdef Pose2Mobile2ArmsModel cyCreateFromShared(const shared_ptr[CPose2Mobile2ArmsModel]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileVetLinArmModel.h" namespace "gpmp2":
    cdef cppclass CPose2MobileVetLinArmModel "gpmp2::Pose2MobileVetLinArmModel":
        CPose2MobileVetLinArmModel(const CPose2MobileVetLinArm& r, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPose2MobileVetLinArm fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const CPose2Vector& conf) except +
        double sphere_radius(size_t i) except +

cdef class Pose2MobileVetLinArmModel:
    cdef shared_ptr[CPose2MobileVetLinArmModel] CPose2MobileVetLinArmModel_
    @staticmethod
    cdef Pose2MobileVetLinArmModel cyCreateFromShared(const shared_ptr[CPose2MobileVetLinArmModel]& other)


cdef extern from "gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h" namespace "gpmp2":
    cdef cppclass CPose2MobileVetLin2ArmsModel "gpmp2::Pose2MobileVetLin2ArmsModel":
        CPose2MobileVetLin2ArmsModel(const CPose2MobileVetLin2Arms& r, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPose2MobileVetLin2Arms fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const CPose2Vector& conf) except +
        double sphere_radius(size_t i) except +

cdef class Pose2MobileVetLin2ArmsModel:
    cdef shared_ptr[CPose2MobileVetLin2ArmsModel] CPose2MobileVetLin2ArmsModel_
    @staticmethod
    cdef Pose2MobileVetLin2ArmsModel cyCreateFromShared(const shared_ptr[CPose2MobileVetLin2ArmsModel]& other)


cdef extern from "gpmp2/kinematics/PointRobotModel.h" namespace "gpmp2":
    cdef cppclass CPointRobotModel "gpmp2::PointRobotModel":
        CPointRobotModel(const CPointRobot& pR, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPointRobot fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const VectorXd& conf) except +
        double sphere_radius(size_t i) except +

cdef class PointRobotModel:
    cdef shared_ptr[CPointRobotModel] CPointRobotModel_
    @staticmethod
    cdef PointRobotModel cyCreateFromShared(const shared_ptr[CPointRobotModel]& other)


cdef extern from "gpmp2/kinematics/GoalFactorArm.h" namespace "gpmp2":
    cdef cppclass CGoalFactorArm "gpmp2::GoalFactorArm"(CNoiseModelFactor):
        CGoalFactorArm(size_t poseKey, const shared_ptr[CnoiseModel_Base]& cost_model, const CArm& arm, const CPoint3& dest_point) except +


cdef class GoalFactorArm(NoiseModelFactor):
    cdef shared_ptr[CGoalFactorArm] CGoalFactorArm_
    @staticmethod
    cdef GoalFactorArm cyCreateFromShared(const shared_ptr[CGoalFactorArm]& other)


cdef extern from "gpmp2/kinematics/JointLimitFactorVector.h" namespace "gpmp2":
    cdef cppclass CJointLimitFactorVector "gpmp2::JointLimitFactorVector"(CNoiseModelFactor):
        CJointLimitFactorVector(size_t key, const shared_ptr[CnoiseModel_Base]& cost_model, const VectorXd& down_limit, const VectorXd& up_limit, const VectorXd& limit_thresh) except +


cdef class JointLimitFactorVector(NoiseModelFactor):
    cdef shared_ptr[CJointLimitFactorVector] CJointLimitFactorVector_
    @staticmethod
    cdef JointLimitFactorVector cyCreateFromShared(const shared_ptr[CJointLimitFactorVector]& other)


cdef extern from "gpmp2/kinematics/VelocityLimitFactorVector.h" namespace "gpmp2":
    cdef cppclass CVelocityLimitFactorVector "gpmp2::VelocityLimitFactorVector"(CNoiseModelFactor):
        CVelocityLimitFactorVector(size_t key, const shared_ptr[CnoiseModel_Base]& cost_model, const VectorXd& vel_limit, const VectorXd& limit_thresh) except +


cdef class VelocityLimitFactorVector(NoiseModelFactor):
    cdef shared_ptr[CVelocityLimitFactorVector] CVelocityLimitFactorVector_
    @staticmethod
    cdef VelocityLimitFactorVector cyCreateFromShared(const shared_ptr[CVelocityLimitFactorVector]& other)


cdef extern from "gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h" namespace "gpmp2":
    cdef cppclass CGaussianPriorWorkspacePositionArm "gpmp2::GaussianPriorWorkspacePositionArm"(CNoiseModelFactor):
        CGaussianPriorWorkspacePositionArm(size_t poseKey, const CArmModel& arm, int joint, const CPoint3& des_position, const shared_ptr[CnoiseModel_Base]& cost_model) except +


cdef class GaussianPriorWorkspacePositionArm(NoiseModelFactor):
    cdef shared_ptr[CGaussianPriorWorkspacePositionArm] CGaussianPriorWorkspacePositionArm_
    @staticmethod
    cdef GaussianPriorWorkspacePositionArm cyCreateFromShared(const shared_ptr[CGaussianPriorWorkspacePositionArm]& other)


cdef extern from "gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h" namespace "gpmp2":
    cdef cppclass CGaussianPriorWorkspaceOrientationArm "gpmp2::GaussianPriorWorkspaceOrientationArm"(CNoiseModelFactor):
        CGaussianPriorWorkspaceOrientationArm(size_t poseKey, const CArmModel& arm, int joint, const CRot3& des_orientation, const shared_ptr[CnoiseModel_Base]& cost_model) except +


cdef class GaussianPriorWorkspaceOrientationArm(NoiseModelFactor):
    cdef shared_ptr[CGaussianPriorWorkspaceOrientationArm] CGaussianPriorWorkspaceOrientationArm_
    @staticmethod
    cdef GaussianPriorWorkspaceOrientationArm cyCreateFromShared(const shared_ptr[CGaussianPriorWorkspaceOrientationArm]& other)


cdef extern from "gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h" namespace "gpmp2":
    cdef cppclass CGaussianPriorWorkspacePoseArm "gpmp2::GaussianPriorWorkspacePoseArm"(CNoiseModelFactor):
        CGaussianPriorWorkspacePoseArm(size_t poseKey, const CArmModel& arm, int joint, const CPose3& des_pose, const shared_ptr[CnoiseModel_Base]& cost_model) except +


cdef class GaussianPriorWorkspacePoseArm(NoiseModelFactor):
    cdef shared_ptr[CGaussianPriorWorkspacePoseArm] CGaussianPriorWorkspacePoseArm_
    @staticmethod
    cdef GaussianPriorWorkspacePoseArm cyCreateFromShared(const shared_ptr[CGaussianPriorWorkspacePoseArm]& other)


cdef extern from "gpmp2/dynamics/VehicleDynamicsFactorPose2.h" namespace "gpmp2":
    cdef cppclass CVehicleDynamicsFactorPose2 "gpmp2::VehicleDynamicsFactorPose2"(CNoiseModelFactor):
        CVehicleDynamicsFactorPose2(size_t poseKey, size_t velKey, double cost_sigma) except +


cdef class VehicleDynamicsFactorPose2(NoiseModelFactor):
    cdef shared_ptr[CVehicleDynamicsFactorPose2] CVehicleDynamicsFactorPose2_
    @staticmethod
    cdef VehicleDynamicsFactorPose2 cyCreateFromShared(const shared_ptr[CVehicleDynamicsFactorPose2]& other)


cdef extern from "gpmp2/dynamics/VehicleDynamicsFactorPose2Vector.h" namespace "gpmp2":
    cdef cppclass CVehicleDynamicsFactorPose2Vector "gpmp2::VehicleDynamicsFactorPose2Vector"(CNoiseModelFactor):
        CVehicleDynamicsFactorPose2Vector(size_t poseKey, size_t velKey, double cost_sigma) except +


cdef class VehicleDynamicsFactorPose2Vector(NoiseModelFactor):
    cdef shared_ptr[CVehicleDynamicsFactorPose2Vector] CVehicleDynamicsFactorPose2Vector_
    @staticmethod
    cdef VehicleDynamicsFactorPose2Vector cyCreateFromShared(const shared_ptr[CVehicleDynamicsFactorPose2Vector]& other)


cdef extern from "gpmp2/dynamics/VehicleDynamicsFactorVector.h" namespace "gpmp2":
    cdef cppclass CVehicleDynamicsFactorVector "gpmp2::VehicleDynamicsFactorVector"(CNoiseModelFactor):
        CVehicleDynamicsFactorVector(size_t poseKey, size_t velKey, double cost_sigma) except +


cdef class VehicleDynamicsFactorVector(NoiseModelFactor):
    cdef shared_ptr[CVehicleDynamicsFactorVector] CVehicleDynamicsFactorVector_
    @staticmethod
    cdef VehicleDynamicsFactorVector cyCreateFromShared(const shared_ptr[CVehicleDynamicsFactorVector]& other)


cdef extern from "gpmp2/obstacle/SignedDistanceField.h" namespace "gpmp2":
    cdef cppclass CSignedDistanceField "gpmp2::SignedDistanceField":
        CSignedDistanceField() except +
        CSignedDistanceField(const CPoint3& origin, double cell_size, size_t field_rows, size_t field_cols, size_t field_z) except +

        double getSignedDistance(const CPoint3& point) except +
        void initFieldData(size_t z_idx, const MatrixXd& field_layer) except +
        void loadSDF(string filename) except +
        void print_ "print"(string s) except +
        void saveSDF(string filename) except +

cdef class SignedDistanceField:
    cdef shared_ptr[CSignedDistanceField] CSignedDistanceField_
    @staticmethod
    cdef SignedDistanceField cyCreateFromShared(const shared_ptr[CSignedDistanceField]& other)


cdef extern from "gpmp2/obstacle/PlanarSDF.h" namespace "gpmp2":
    cdef cppclass CPlanarSDF "gpmp2::PlanarSDF":
        CPlanarSDF(const CPoint2& origin, double cell_size, const MatrixXd& data) except +

        double getSignedDistance(const CPoint2& point) except +
        void print_ "print"(string s) except +

cdef class PlanarSDF:
    cdef shared_ptr[CPlanarSDF] CPlanarSDF_
    @staticmethod
    cdef PlanarSDF cyCreateFromShared(const shared_ptr[CPlanarSDF]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorArm.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorArm "gpmp2::ObstacleSDFFactorArm"(CNoiseModelFactor):
        CObstacleSDFFactorArm(size_t poseKey, const CArmModel& arm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const VectorXd& pose) except +

cdef class ObstacleSDFFactorArm(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorArm] CObstacleSDFFactorArm_
    @staticmethod
    cdef ObstacleSDFFactorArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorArm]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorGPArm.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorGPArm "gpmp2::ObstacleSDFFactorGPArm"(CNoiseModelFactor):
        CObstacleSDFFactorGPArm(size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key, const CArmModel& arm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstacleSDFFactorGPArm(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorGPArm] CObstacleSDFFactorGPArm_
    @staticmethod
    cdef ObstacleSDFFactorGPArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPArm]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorArm "gpmp2::ObstaclePlanarSDFFactorArm"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorArm(size_t posekey, const CArmModel& arm, const CPlanarSDF& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const VectorXd& pose) except +

cdef class ObstaclePlanarSDFFactorArm(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorArm] CObstaclePlanarSDFFactorArm_
    @staticmethod
    cdef ObstaclePlanarSDFFactorArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorArm]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorGPArm "gpmp2::ObstaclePlanarSDFFactorGPArm"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorGPArm(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CArmModel& arm, const CPlanarSDF& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstaclePlanarSDFFactorGPArm(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorGPArm] CObstaclePlanarSDFFactorGPArm_
    @staticmethod
    cdef ObstaclePlanarSDFFactorGPArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPArm]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorPointRobot "gpmp2::ObstaclePlanarSDFFactorPointRobot"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorPointRobot(size_t posekey, const CPointRobotModel& pR, const CPlanarSDF& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const VectorXd& pose) except +

cdef class ObstaclePlanarSDFFactorPointRobot(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorPointRobot] CObstaclePlanarSDFFactorPointRobot_
    @staticmethod
    cdef ObstaclePlanarSDFFactorPointRobot cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPointRobot]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorGPPointRobot "gpmp2::ObstaclePlanarSDFFactorGPPointRobot"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorGPPointRobot(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPointRobotModel& pR, const CPlanarSDF& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstaclePlanarSDFFactorGPPointRobot(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorGPPointRobot] CObstaclePlanarSDFFactorGPPointRobot_
    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPointRobot cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPointRobot]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorPose2MobileBase "gpmp2::ObstaclePlanarSDFFactorPose2MobileBase"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorPose2MobileBase(size_t posekey, const CPose2MobileBaseModel& robot, const CPlanarSDF& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2& pose) except +

cdef class ObstaclePlanarSDFFactorPose2MobileBase(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase] CObstaclePlanarSDFFactorPose2MobileBase_
    @staticmethod
    cdef ObstaclePlanarSDFFactorPose2MobileBase cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorGPPose2MobileBase "gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorGPPose2MobileBase(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2MobileBaseModel& robot, const CPlanarSDF& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstaclePlanarSDFFactorGPPose2MobileBase(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase] CObstaclePlanarSDFFactorGPPose2MobileBase_
    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPose2MobileBase cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorPose2MobileArm "gpmp2::ObstaclePlanarSDFFactorPose2MobileArm"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorPose2MobileArm(size_t posekey, const CPose2MobileArmModel& marm, const CPlanarSDF& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2Vector& pose) except +

cdef class ObstaclePlanarSDFFactorPose2MobileArm(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm] CObstaclePlanarSDFFactorPose2MobileArm_
    @staticmethod
    cdef ObstaclePlanarSDFFactorPose2MobileArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorGPPose2MobileArm "gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorGPPose2MobileArm(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2MobileArmModel& marm, const CPlanarSDF& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstaclePlanarSDFFactorGPPose2MobileArm(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm] CObstaclePlanarSDFFactorGPPose2MobileArm_
    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPose2MobileArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorPose2Mobile2Arms.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorPose2Mobile2Arms "gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorPose2Mobile2Arms(size_t posekey, const CPose2Mobile2ArmsModel& marm, const CPlanarSDF& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2Vector& pose) except +

cdef class ObstaclePlanarSDFFactorPose2Mobile2Arms(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms] CObstaclePlanarSDFFactorPose2Mobile2Arms_
    @staticmethod
    cdef ObstaclePlanarSDFFactorPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2Mobile2Arms.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorGPPose2Mobile2Arms "gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorGPPose2Mobile2Arms(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2Mobile2ArmsModel& marm, const CPlanarSDF& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstaclePlanarSDFFactorGPPose2Mobile2Arms(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms] CObstaclePlanarSDFFactorGPPose2Mobile2Arms_
    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorPose2MobileBase "gpmp2::ObstacleSDFFactorPose2MobileBase"(CNoiseModelFactor):
        CObstacleSDFFactorPose2MobileBase(size_t posekey, const CPose2MobileBaseModel& robot, const CSignedDistanceField& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2& pose) except +

cdef class ObstacleSDFFactorPose2MobileBase(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorPose2MobileBase] CObstacleSDFFactorPose2MobileBase_
    @staticmethod
    cdef ObstacleSDFFactorPose2MobileBase cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileBase]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorGPPose2MobileBase "gpmp2::ObstacleSDFFactorGPPose2MobileBase"(CNoiseModelFactor):
        CObstacleSDFFactorGPPose2MobileBase(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2MobileBaseModel& robot, const CSignedDistanceField& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstacleSDFFactorGPPose2MobileBase(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorGPPose2MobileBase] CObstacleSDFFactorGPPose2MobileBase_
    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileBase cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileBase]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorPose2MobileArm "gpmp2::ObstacleSDFFactorPose2MobileArm"(CNoiseModelFactor):
        CObstacleSDFFactorPose2MobileArm(size_t posekey, const CPose2MobileArmModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2Vector& pose) except +

cdef class ObstacleSDFFactorPose2MobileArm(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorPose2MobileArm] CObstacleSDFFactorPose2MobileArm_
    @staticmethod
    cdef ObstacleSDFFactorPose2MobileArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileArm]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorGPPose2MobileArm "gpmp2::ObstacleSDFFactorGPPose2MobileArm"(CNoiseModelFactor):
        CObstacleSDFFactorGPPose2MobileArm(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2MobileArmModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstacleSDFFactorGPPose2MobileArm(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorGPPose2MobileArm] CObstacleSDFFactorGPPose2MobileArm_
    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileArm]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorPose2Mobile2Arms "gpmp2::ObstacleSDFFactorPose2Mobile2Arms"(CNoiseModelFactor):
        CObstacleSDFFactorPose2Mobile2Arms(size_t posekey, const CPose2Mobile2ArmsModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2Vector& pose) except +

cdef class ObstacleSDFFactorPose2Mobile2Arms(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorPose2Mobile2Arms] CObstacleSDFFactorPose2Mobile2Arms_
    @staticmethod
    cdef ObstacleSDFFactorPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2Mobile2Arms]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorGPPose2Mobile2Arms "gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms"(CNoiseModelFactor):
        CObstacleSDFFactorGPPose2Mobile2Arms(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2Mobile2ArmsModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstacleSDFFactorGPPose2Mobile2Arms(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms] CObstacleSDFFactorGPPose2Mobile2Arms_
    @staticmethod
    cdef ObstacleSDFFactorGPPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorPose2MobileVetLinArm "gpmp2::ObstacleSDFFactorPose2MobileVetLinArm"(CNoiseModelFactor):
        CObstacleSDFFactorPose2MobileVetLinArm(size_t posekey, const CPose2MobileVetLinArmModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2Vector& pose) except +

cdef class ObstacleSDFFactorPose2MobileVetLinArm(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm] CObstacleSDFFactorPose2MobileVetLinArm_
    @staticmethod
    cdef ObstacleSDFFactorPose2MobileVetLinArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorGPPose2MobileVetLinArm "gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm"(CNoiseModelFactor):
        CObstacleSDFFactorGPPose2MobileVetLinArm(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2MobileVetLinArmModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstacleSDFFactorGPPose2MobileVetLinArm(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm] CObstacleSDFFactorGPPose2MobileVetLinArm_
    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileVetLinArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorPose2MobileVetLin2Arms "gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms"(CNoiseModelFactor):
        CObstacleSDFFactorPose2MobileVetLin2Arms(size_t posekey, const CPose2MobileVetLin2ArmsModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const CPose2Vector& pose) except +

cdef class ObstacleSDFFactorPose2MobileVetLin2Arms(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms] CObstacleSDFFactorPose2MobileVetLin2Arms_
    @staticmethod
    cdef ObstacleSDFFactorPose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms]& other)


cdef extern from "gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h" namespace "gpmp2":
    cdef cppclass CObstacleSDFFactorGPPose2MobileVetLin2Arms "gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms"(CNoiseModelFactor):
        CObstacleSDFFactorGPPose2MobileVetLin2Arms(size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key, const CPose2MobileVetLin2ArmsModel& marm, const CSignedDistanceField& sdf, double cost_sigma, double epsilon, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, double tau) except +


cdef class ObstacleSDFFactorGPPose2MobileVetLin2Arms(NoiseModelFactor):
    cdef shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms] CObstacleSDFFactorGPPose2MobileVetLin2Arms_
    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms]& other)


cdef extern from "gpmp2/obstacle/SelfCollisionArm.h" namespace "gpmp2":
    cdef cppclass CSelfCollisionArm "gpmp2::SelfCollisionArm"(CNoiseModelFactor):
        CSelfCollisionArm(size_t poseKey, const CArmModel& arm, const MatrixXd& data) except +

        VectorXd evaluateError(const VectorXd& pose) except +

cdef class SelfCollisionArm(NoiseModelFactor):
    cdef shared_ptr[CSelfCollisionArm] CSelfCollisionArm_
    @staticmethod
    cdef SelfCollisionArm cyCreateFromShared(const shared_ptr[CSelfCollisionArm]& other)


cdef extern from "gpmp2/planner/TrajOptimizerSetting.h" namespace "gpmp2":
    cdef cppclass CTrajOptimizerSetting "gpmp2::TrajOptimizerSetting":
        CTrajOptimizerSetting(size_t dof) except +

        void setDogleg() except +
        void setGaussNewton() except +
        void setLM() except +
        void setOptimizationNoIncrase(bool flag) except +
        void setVerbosityError() except +
        void setVerbosityNone() except +
        void set_Qc_model(const MatrixXd& Qc) except +
        void set_conf_prior_model(double sigma) except +
        void set_cost_sigma(double sigma) except +
        void set_epsilon(double eps) except +
        void set_flag_pos_limit(bool flag) except +
        void set_flag_vel_limit(bool flag) except +
        void set_joint_pos_limits_down(const VectorXd& v) except +
        void set_joint_pos_limits_up(const VectorXd& v) except +
        void set_max_iter(size_t iter) except +
        void set_obs_check_inter(size_t inter) except +
        void set_pos_limit_model(const VectorXd& v) except +
        void set_pos_limit_thresh(const VectorXd& v) except +
        void set_rel_thresh(double thresh) except +
        void set_total_step(size_t step) except +
        void set_total_time(double time) except +
        void set_vel_limit_model(const VectorXd& v) except +
        void set_vel_limit_thresh(const VectorXd& v) except +
        void set_vel_limits(const VectorXd& v) except +
        void set_vel_prior_model(double sigma) except +

cdef class TrajOptimizerSetting:
    cdef shared_ptr[CTrajOptimizerSetting] CTrajOptimizerSetting_
    @staticmethod
    cdef TrajOptimizerSetting cyCreateFromShared(const shared_ptr[CTrajOptimizerSetting]& other)


cdef extern from "gpmp2/planner/ISAM2TrajOptimizer.h" namespace "gpmp2":
    cdef cppclass CISAM2TrajOptimizer2DArm "gpmp2::ISAM2TrajOptimizer2DArm":
        CISAM2TrajOptimizer2DArm(const CArmModel& arm, const CPlanarSDF& sdf, const CTrajOptimizerSetting& setting) except +

        void addPoseEstimate(size_t state_idx, const VectorXd& pose, const MatrixXd& pose_cov) except +
        void addStateEstimate(size_t state_idx, const VectorXd& pose, const MatrixXd& pose_cov, const VectorXd& vel, const MatrixXd& vel_cov) except +
        void changeGoalConfigAndVel(const VectorXd& goal_conf, const VectorXd& goal_vel) except +
        void fixConfigAndVel(size_t state_idx, const VectorXd& conf_fix, const VectorXd& vel_fix) except +
        void initFactorGraph(const VectorXd& start_conf, const VectorXd& start_vel, const VectorXd& goal_conf, const VectorXd& goal_vel) except +
        void initValues(const CValues& init_values) except +
        void removeGoalConfigAndVel() except +
        void update() except +
        CValues values() except +

cdef class ISAM2TrajOptimizer2DArm:
    cdef shared_ptr[CISAM2TrajOptimizer2DArm] CISAM2TrajOptimizer2DArm_
    @staticmethod
    cdef ISAM2TrajOptimizer2DArm cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizer2DArm]& other)


cdef extern from "gpmp2/planner/ISAM2TrajOptimizer.h" namespace "gpmp2":
    cdef cppclass CISAM2TrajOptimizer3DArm "gpmp2::ISAM2TrajOptimizer3DArm":
        CISAM2TrajOptimizer3DArm(const CArmModel& arm, const CSignedDistanceField& sdf, const CTrajOptimizerSetting& setting) except +

        void addPoseEstimate(size_t state_idx, const VectorXd& pose, const MatrixXd& pose_cov) except +
        void addStateEstimate(size_t state_idx, const VectorXd& pose, const MatrixXd& pose_cov, const VectorXd& vel, const MatrixXd& vel_cov) except +
        void changeGoalConfigAndVel(const VectorXd& goal_conf, const VectorXd& goal_vel) except +
        void fixConfigAndVel(size_t state_idx, const VectorXd& conf_fix, const VectorXd& vel_fix) except +
        void initFactorGraph(const VectorXd& start_conf, const VectorXd& start_vel, const VectorXd& goal_conf, const VectorXd& goal_vel) except +
        void initValues(const CValues& init_values) except +
        void removeGoalConfigAndVel() except +
        void update() except +
        CValues values() except +

cdef class ISAM2TrajOptimizer3DArm:
    cdef shared_ptr[CISAM2TrajOptimizer3DArm] CISAM2TrajOptimizer3DArm_
    @staticmethod
    cdef ISAM2TrajOptimizer3DArm cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizer3DArm]& other)


cdef extern from "gpmp2/planner/ISAM2TrajOptimizer.h" namespace "gpmp2":
    cdef cppclass CISAM2TrajOptimizerPose2MobileArm2D "gpmp2::ISAM2TrajOptimizerPose2MobileArm2D":
        CISAM2TrajOptimizerPose2MobileArm2D(const CPose2MobileArmModel& marm, const CPlanarSDF& sdf, const CTrajOptimizerSetting& setting) except +

        void addPoseEstimate(size_t state_idx, const CPose2Vector& pose, const MatrixXd& pose_cov) except +
        void addStateEstimate(size_t state_idx, const CPose2Vector& pose, const MatrixXd& pose_cov, const VectorXd& vel, const MatrixXd& vel_cov) except +
        void changeGoalConfigAndVel(const CPose2Vector& goal_conf, const VectorXd& goal_vel) except +
        void fixConfigAndVel(size_t state_idx, const CPose2Vector& conf_fix, const VectorXd& vel_fix) except +
        void initFactorGraph(const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& goal_conf, const VectorXd& goal_vel) except +
        void initValues(const CValues& init_values) except +
        void removeGoalConfigAndVel() except +
        void update() except +
        CValues values() except +

cdef class ISAM2TrajOptimizerPose2MobileArm2D:
    cdef shared_ptr[CISAM2TrajOptimizerPose2MobileArm2D] CISAM2TrajOptimizerPose2MobileArm2D_
    @staticmethod
    cdef ISAM2TrajOptimizerPose2MobileArm2D cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizerPose2MobileArm2D]& other)


cdef extern from "gpmp2/planner/ISAM2TrajOptimizer.h" namespace "gpmp2":
    cdef cppclass CISAM2TrajOptimizerPose2MobileArm "gpmp2::ISAM2TrajOptimizerPose2MobileArm":
        CISAM2TrajOptimizerPose2MobileArm(const CPose2MobileArmModel& marm, const CSignedDistanceField& sdf, const CTrajOptimizerSetting& setting) except +

        void addPoseEstimate(size_t state_idx, const CPose2Vector& pose, const MatrixXd& pose_cov) except +
        void addStateEstimate(size_t state_idx, const CPose2Vector& pose, const MatrixXd& pose_cov, const VectorXd& vel, const MatrixXd& vel_cov) except +
        void changeGoalConfigAndVel(const CPose2Vector& goal_conf, const VectorXd& goal_vel) except +
        void fixConfigAndVel(size_t state_idx, const CPose2Vector& conf_fix, const VectorXd& vel_fix) except +
        void initFactorGraph(const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& goal_conf, const VectorXd& goal_vel) except +
        void initValues(const CValues& init_values) except +
        void removeGoalConfigAndVel() except +
        void update() except +
        CValues values() except +

cdef class ISAM2TrajOptimizerPose2MobileArm:
    cdef shared_ptr[CISAM2TrajOptimizerPose2MobileArm] CISAM2TrajOptimizerPose2MobileArm_
    @staticmethod
    cdef ISAM2TrajOptimizerPose2MobileArm cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizerPose2MobileArm]& other)


cdef extern from "gpmp2/planner/ISAM2TrajOptimizer.h" namespace "gpmp2":
    cdef cppclass CISAM2TrajOptimizerPose2MobileVetLin2Arms "gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms":
        CISAM2TrajOptimizerPose2MobileVetLin2Arms(const CPose2MobileVetLin2ArmsModel& marm, const CSignedDistanceField& sdf, const CTrajOptimizerSetting& setting) except +

        void addPoseEstimate(size_t state_idx, const CPose2Vector& pose, const MatrixXd& pose_cov) except +
        void addStateEstimate(size_t state_idx, const CPose2Vector& pose, const MatrixXd& pose_cov, const VectorXd& vel, const MatrixXd& vel_cov) except +
        void changeGoalConfigAndVel(const CPose2Vector& goal_conf, const VectorXd& goal_vel) except +
        void fixConfigAndVel(size_t state_idx, const CPose2Vector& conf_fix, const VectorXd& vel_fix) except +
        void initFactorGraph(const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& goal_conf, const VectorXd& goal_vel) except +
        void initValues(const CValues& init_values) except +
        void removeGoalConfigAndVel() except +
        void update() except +
        CValues values() except +

cdef class ISAM2TrajOptimizerPose2MobileVetLin2Arms:
    cdef shared_ptr[CISAM2TrajOptimizerPose2MobileVetLin2Arms] CISAM2TrajOptimizerPose2MobileVetLin2Arms_
    @staticmethod
    cdef ISAM2TrajOptimizerPose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizerPose2MobileVetLin2Arms]& other)


cdef extern from "gpmp2/utils/matlabUtils.h" namespace "gpmp2":
    cdef cppclass CPriorFactorPose2Vector "gpmp2::PriorFactorPose2Vector"(CNoiseModelFactor):
        CPriorFactorPose2Vector(size_t poseKey, const CPose2Vector& value, const shared_ptr[CnoiseModel_Base]& model) except +


cdef class PriorFactorPose2Vector(NoiseModelFactor):
    cdef shared_ptr[CPriorFactorPose2Vector] CPriorFactorPose2Vector_
    @staticmethod
    cdef PriorFactorPose2Vector cyCreateFromShared(const shared_ptr[CPriorFactorPose2Vector]& other)


cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimize2DArm "gpmp2::BatchTrajOptimize2DArm"(const CArmModel& arm, const CPlanarSDF& sdf, const VectorXd& start_conf, const VectorXd& start_vel, const VectorXd& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimize3DArm "gpmp2::BatchTrajOptimize3DArm"(const CArmModel& arm, const CSignedDistanceField& sdf, const VectorXd& start_conf, const VectorXd& start_vel, const VectorXd& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimizePose2Mobile2Arms "gpmp2::BatchTrajOptimizePose2Mobile2Arms"(const CPose2Mobile2ArmsModel& marm, const CSignedDistanceField& sdf, const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimizePose2MobileArm "gpmp2::BatchTrajOptimizePose2MobileArm"(const CPose2MobileArmModel& marm, const CSignedDistanceField& sdf, const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimizePose2MobileArm2D "gpmp2::BatchTrajOptimizePose2MobileArm2D"(const CPose2MobileArmModel& marm, const CPlanarSDF& sdf, const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimizePose2MobileVetLin2Arms "gpmp2::BatchTrajOptimizePose2MobileVetLin2Arms"(const CPose2MobileVetLin2ArmsModel& marm, const CSignedDistanceField& sdf, const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_BatchTrajOptimizePose2MobileVetLinArm "gpmp2::BatchTrajOptimizePose2MobileVetLinArm"(const CPose2MobileVetLinArmModel& marm, const CSignedDistanceField& sdf, const CPose2Vector& start_conf, const VectorXd& start_vel, const CPose2Vector& end_conf, const VectorXd& end_vel, const CValues& init_values, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCost2DArm "gpmp2::CollisionCost2DArm"(const CArmModel& arm, const CPlanarSDF& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCost3DArm "gpmp2::CollisionCost3DArm"(const CArmModel& arm, const CSignedDistanceField& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2Mobile2Arms "gpmp2::CollisionCostPose2Mobile2Arms"(const CPose2Mobile2ArmsModel& marm, const CSignedDistanceField& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2MobileArm "gpmp2::CollisionCostPose2MobileArm"(const CPose2MobileArmModel& marm, const CSignedDistanceField& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2MobileArm2D "gpmp2::CollisionCostPose2MobileArm2D"(const CPose2MobileArmModel& marm, const CPlanarSDF& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2MobileBase "gpmp2::CollisionCostPose2MobileBase"(const CPose2MobileBaseModel& robot, const CSignedDistanceField& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2MobileBase2D "gpmp2::CollisionCostPose2MobileBase2D"(const CPose2MobileBaseModel& robot, const CPlanarSDF& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2MobileVetLin2Arms "gpmp2::CollisionCostPose2MobileVetLin2Arms"(const CPose2MobileVetLin2ArmsModel& marm, const CSignedDistanceField& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        double pxd_CollisionCostPose2MobileVetLinArm "gpmp2::CollisionCostPose2MobileVetLinArm"(const CPose2MobileVetLinArmModel& marm, const CSignedDistanceField& sdf, const CValues& result, const CTrajOptimizerSetting& setting)
cdef extern from "gpmp2/utils/matlabUtils.h" namespace "gpmp2":
        CPose2Vector pxd_atPose2VectorValues "gpmp2::atPose2VectorValues"(size_t key, const CValues& values)
cdef extern from "gpmp2/planner/TrajUtils.h" namespace "gpmp2":
        CValues pxd_initArmTrajStraightLine "gpmp2::initArmTrajStraightLine"(const VectorXd& init_conf, const VectorXd& end_conf, size_t total_step)
cdef extern from "gpmp2/planner/TrajUtils.h" namespace "gpmp2":
        CValues pxd_initPose2TrajStraightLine "gpmp2::initPose2TrajStraightLine"(const CPose2& init_pose, const CPose2& end_pose, size_t total_step)
cdef extern from "gpmp2/planner/TrajUtils.h" namespace "gpmp2":
        CValues pxd_initPose2VectorTrajStraightLine "gpmp2::initPose2VectorTrajStraightLine"(const CPose2& init_pose, const VectorXd& init_conf, const CPose2& end_pose, const VectorXd& end_conf, size_t total_step)
cdef extern from "gpmp2/utils/matlabUtils.h" namespace "gpmp2":
        void pxd_insertPose2VectorInValues "gpmp2::insertPose2VectorInValues"(size_t key, const CPose2Vector& p, CValues& values)
cdef extern from "gpmp2/planner/TrajUtils.h" namespace "gpmp2":
        CValues pxd_interpolateArmTraj "gpmp2::interpolateArmTraj"(const CValues& opt_values, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, size_t inter_step)
        CValues pxd_interpolateArmTraj "gpmp2::interpolateArmTraj"(const CValues& opt_values, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, size_t inter_step, size_t start_index, size_t end_index)
cdef extern from "gpmp2/planner/TrajUtils.h" namespace "gpmp2":
        CValues pxd_interpolatePose2MobileArmTraj "gpmp2::interpolatePose2MobileArmTraj"(const CValues& opt_values, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, size_t inter_step, size_t start_index, size_t end_index)
cdef extern from "gpmp2/planner/TrajUtils.h" namespace "gpmp2":
        CValues pxd_interpolatePose2Traj "gpmp2::interpolatePose2Traj"(const CValues& opt_values, const shared_ptr[CnoiseModel_Base]& Qc_model, double delta_t, size_t inter_step, size_t start_index, size_t end_index)
cdef extern from "gpmp2/planner/BatchTrajOptimizer.h" namespace "gpmp2":
        CValues pxd_optimize "gpmp2::optimize"(const CNonlinearFactorGraph& graph, const CValues& init_values, const CTrajOptimizerSetting& setting, bool iter_no_increase)

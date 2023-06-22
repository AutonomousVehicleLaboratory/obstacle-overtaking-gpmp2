cimport numpy as np
import numpy as npp
cimport gpmp2
from gpmp2 cimport shared_ptr
from gpmp2 cimport dynamic_pointer_cast
from gpmp2 cimport make_shared
# C helper function that copies all arguments into a positional list.
cdef list process_args(list keywords, tuple args, dict kwargs):
   cdef str keyword
   cdef int n = len(args), m = len(keywords)
   cdef list params = list(args)
   assert len(args)+len(kwargs) == m, 'Expected {} arguments'.format(m)
   try:
       return params + [kwargs[keyword] for keyword in keywords[n:]]
   except:
       raise ValueError('Epected arguments ' + str(keywords))
from gtsam_eigency.core cimport *
from libcpp cimport bool

from libcpp.pair cimport pair
from libcpp.string cimport string
from cython.operator cimport dereference as deref


cdef class Pose2Vector:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2Vector_ = shared_ptr[CPose2Vector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CPose2Vector_ = shared_ptr[CPose2Vector](new CPose2Vector())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['pose', 'c'], args, kwargs)
            pose = <Pose2>(__params[0])
            c = <np.ndarray>(__params[1])
            assert isinstance(pose, Pose2)
            assert isinstance(c, np.ndarray) and c.ndim == 1
            c = c.astype(float, order='F', copy=False)
            self.CPose2Vector_ = shared_ptr[CPose2Vector](new CPose2Vector(deref(pose.CPose2_), <VectorXd>(Map[VectorXd](c))))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2Vector_.use_count()==0):
            raise TypeError('Pose2Vector construction failed!')

    @staticmethod
    cdef Pose2Vector cyCreateFromShared(const shared_ptr[CPose2Vector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2Vector return_value = Pose2Vector(cyCreateFromShared=True)
        return_value.CPose2Vector_ = other
        return return_value

    def configuration(self):
        cdef VectorXd ret = self.CPose2Vector_.get().configuration()
        return ndarray_copy(ret).squeeze()
    def pose(self):
        cdef shared_ptr[CPose2] ret = make_shared[CPose2](self.CPose2Vector_.get().pose())
        return Pose2.cyCreateFromShared(ret)
    def __str__(self):
        strBuf = RedirectCout()
        self.print_('')
        return strBuf.str()
    def print_(self, string s):
        self.CPose2Vector_.get().print_(s)


cdef class GaussianProcessPriorLinear(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianProcessPriorLinear_ = shared_ptr[CGaussianProcessPriorLinear]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['key1', 'key2', 'key3', 'key4', 'delta', 'Qc_model'], args, kwargs)
            key1 = <size_t>(__params[0])
            key2 = <size_t>(__params[1])
            key3 = <size_t>(__params[2])
            key4 = <size_t>(__params[3])
            delta = <double>(__params[4])
            Qc_model = <noiseModel_Base>(__params[5])
            assert isinstance(Qc_model, noiseModel_Base)
            self.CGaussianProcessPriorLinear_ = shared_ptr[CGaussianProcessPriorLinear](new CGaussianProcessPriorLinear(key1, key2, key3, key4, delta, Qc_model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianProcessPriorLinear_.use_count()==0):
            raise TypeError('GaussianProcessPriorLinear construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGaussianProcessPriorLinear_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGaussianProcessPriorLinear_)

    @staticmethod
    cdef GaussianProcessPriorLinear cyCreateFromShared(const shared_ptr[CGaussianProcessPriorLinear]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianProcessPriorLinear return_value = GaussianProcessPriorLinear(cyCreateFromShared=True)
        return_value.CGaussianProcessPriorLinear_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, np.ndarray pose1, np.ndarray vel1, np.ndarray pose2, np.ndarray vel2):
        pose1 = pose1.astype(float, order='F', copy=False)
        vel1 = vel1.astype(float, order='F', copy=False)
        pose2 = pose2.astype(float, order='F', copy=False)
        vel2 = vel2.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CGaussianProcessPriorLinear_.get().evaluateError(<VectorXd>(Map[VectorXd](pose1)), <VectorXd>(Map[VectorXd](vel1)), <VectorXd>(Map[VectorXd](pose2)), <VectorXd>(Map[VectorXd](vel2)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_GaussianProcessPriorLinear_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GaussianProcessPriorLinear.cyCreateFromShared(<shared_ptr[CGaussianProcessPriorLinear]>dynamic_pointer_cast[CGaussianProcessPriorLinear,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GaussianProcessPriorLinear_NonlinearFactor(NonlinearFactor parent):
    try:
        return GaussianProcessPriorLinear.cyCreateFromShared(<shared_ptr[CGaussianProcessPriorLinear]>dynamic_pointer_cast[CGaussianProcessPriorLinear,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class GaussianProcessPriorPose2(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianProcessPriorPose2_ = shared_ptr[CGaussianProcessPriorPose2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['key1', 'key2', 'key3', 'key4', 'delta', 'Qc_model'], args, kwargs)
            key1 = <size_t>(__params[0])
            key2 = <size_t>(__params[1])
            key3 = <size_t>(__params[2])
            key4 = <size_t>(__params[3])
            delta = <double>(__params[4])
            Qc_model = <noiseModel_Base>(__params[5])
            assert isinstance(Qc_model, noiseModel_Base)
            self.CGaussianProcessPriorPose2_ = shared_ptr[CGaussianProcessPriorPose2](new CGaussianProcessPriorPose2(key1, key2, key3, key4, delta, Qc_model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianProcessPriorPose2_.use_count()==0):
            raise TypeError('GaussianProcessPriorPose2 construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGaussianProcessPriorPose2_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGaussianProcessPriorPose2_)

    @staticmethod
    cdef GaussianProcessPriorPose2 cyCreateFromShared(const shared_ptr[CGaussianProcessPriorPose2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianProcessPriorPose2 return_value = GaussianProcessPriorPose2(cyCreateFromShared=True)
        return_value.CGaussianProcessPriorPose2_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_GaussianProcessPriorPose2_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GaussianProcessPriorPose2.cyCreateFromShared(<shared_ptr[CGaussianProcessPriorPose2]>dynamic_pointer_cast[CGaussianProcessPriorPose2,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GaussianProcessPriorPose2_NonlinearFactor(NonlinearFactor parent):
    try:
        return GaussianProcessPriorPose2.cyCreateFromShared(<shared_ptr[CGaussianProcessPriorPose2]>dynamic_pointer_cast[CGaussianProcessPriorPose2,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class GaussianProcessPriorPose2Vector(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianProcessPriorPose2Vector_ = shared_ptr[CGaussianProcessPriorPose2Vector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['key1', 'key2', 'key3', 'key4', 'delta', 'Qc_model'], args, kwargs)
            key1 = <size_t>(__params[0])
            key2 = <size_t>(__params[1])
            key3 = <size_t>(__params[2])
            key4 = <size_t>(__params[3])
            delta = <double>(__params[4])
            Qc_model = <noiseModel_Base>(__params[5])
            assert isinstance(Qc_model, noiseModel_Base)
            self.CGaussianProcessPriorPose2Vector_ = shared_ptr[CGaussianProcessPriorPose2Vector](new CGaussianProcessPriorPose2Vector(key1, key2, key3, key4, delta, Qc_model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianProcessPriorPose2Vector_.use_count()==0):
            raise TypeError('GaussianProcessPriorPose2Vector construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGaussianProcessPriorPose2Vector_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGaussianProcessPriorPose2Vector_)

    @staticmethod
    cdef GaussianProcessPriorPose2Vector cyCreateFromShared(const shared_ptr[CGaussianProcessPriorPose2Vector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianProcessPriorPose2Vector return_value = GaussianProcessPriorPose2Vector(cyCreateFromShared=True)
        return_value.CGaussianProcessPriorPose2Vector_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_GaussianProcessPriorPose2Vector_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GaussianProcessPriorPose2Vector.cyCreateFromShared(<shared_ptr[CGaussianProcessPriorPose2Vector]>dynamic_pointer_cast[CGaussianProcessPriorPose2Vector,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GaussianProcessPriorPose2Vector_NonlinearFactor(NonlinearFactor parent):
    try:
        return GaussianProcessPriorPose2Vector.cyCreateFromShared(<shared_ptr[CGaussianProcessPriorPose2Vector]>dynamic_pointer_cast[CGaussianProcessPriorPose2Vector,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class GaussianProcessInterpolatorLinear:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianProcessInterpolatorLinear_ = shared_ptr[CGaussianProcessInterpolatorLinear]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['Qc_model', 'delta_t', 'tau'], args, kwargs)
            Qc_model = <noiseModel_Base>(__params[0])
            delta_t = <double>(__params[1])
            tau = <double>(__params[2])
            assert isinstance(Qc_model, noiseModel_Base)
            self.CGaussianProcessInterpolatorLinear_ = shared_ptr[CGaussianProcessInterpolatorLinear](new CGaussianProcessInterpolatorLinear(Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianProcessInterpolatorLinear_.use_count()==0):
            raise TypeError('GaussianProcessInterpolatorLinear construction failed!')

    @staticmethod
    cdef GaussianProcessInterpolatorLinear cyCreateFromShared(const shared_ptr[CGaussianProcessInterpolatorLinear]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianProcessInterpolatorLinear return_value = GaussianProcessInterpolatorLinear(cyCreateFromShared=True)
        return_value.CGaussianProcessInterpolatorLinear_ = other
        return return_value

    def interpolatePose(self, np.ndarray pose1, np.ndarray vel1, np.ndarray pose2, np.ndarray vel2):
        pose1 = pose1.astype(float, order='F', copy=False)
        vel1 = vel1.astype(float, order='F', copy=False)
        pose2 = pose2.astype(float, order='F', copy=False)
        vel2 = vel2.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CGaussianProcessInterpolatorLinear_.get().interpolatePose(<VectorXd>(Map[VectorXd](pose1)), <VectorXd>(Map[VectorXd](vel1)), <VectorXd>(Map[VectorXd](pose2)), <VectorXd>(Map[VectorXd](vel2)))
        return ndarray_copy(ret).squeeze()
    def interpolateVelocity(self, np.ndarray pose1, np.ndarray vel1, np.ndarray pose2, np.ndarray vel2):
        pose1 = pose1.astype(float, order='F', copy=False)
        vel1 = vel1.astype(float, order='F', copy=False)
        pose2 = pose2.astype(float, order='F', copy=False)
        vel2 = vel2.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CGaussianProcessInterpolatorLinear_.get().interpolateVelocity(<VectorXd>(Map[VectorXd](pose1)), <VectorXd>(Map[VectorXd](vel1)), <VectorXd>(Map[VectorXd](pose2)), <VectorXd>(Map[VectorXd](vel2)))
        return ndarray_copy(ret).squeeze()


cdef class Arm:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CArm_ = shared_ptr[CArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['dof', 'a', 'alpha', 'd'], args, kwargs)
            dof = <size_t>(__params[0])
            a = <np.ndarray>(__params[1])
            alpha = <np.ndarray>(__params[2])
            d = <np.ndarray>(__params[3])
            assert isinstance(a, np.ndarray) and a.ndim == 1
            assert isinstance(alpha, np.ndarray) and alpha.ndim == 1
            assert isinstance(d, np.ndarray) and d.ndim == 1
            a = a.astype(float, order='F', copy=False)
            alpha = alpha.astype(float, order='F', copy=False)
            d = d.astype(float, order='F', copy=False)
            self.CArm_ = shared_ptr[CArm](new CArm(dof, <VectorXd>(Map[VectorXd](a)), <VectorXd>(Map[VectorXd](alpha)), <VectorXd>(Map[VectorXd](d))))
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['dof', 'a', 'alpha', 'd', 'base_pose'], args, kwargs)
            dof = <size_t>(__params[0])
            a = <np.ndarray>(__params[1])
            alpha = <np.ndarray>(__params[2])
            d = <np.ndarray>(__params[3])
            base_pose = <Pose3>(__params[4])
            assert isinstance(a, np.ndarray) and a.ndim == 1
            assert isinstance(alpha, np.ndarray) and alpha.ndim == 1
            assert isinstance(d, np.ndarray) and d.ndim == 1
            assert isinstance(base_pose, Pose3)
            a = a.astype(float, order='F', copy=False)
            alpha = alpha.astype(float, order='F', copy=False)
            d = d.astype(float, order='F', copy=False)
            self.CArm_ = shared_ptr[CArm](new CArm(dof, <VectorXd>(Map[VectorXd](a)), <VectorXd>(Map[VectorXd](alpha)), <VectorXd>(Map[VectorXd](d)), deref(base_pose.CPose3_)))
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['dof', 'a', 'alpha', 'd', 'base_pose', 'theta_bias'], args, kwargs)
            dof = <size_t>(__params[0])
            a = <np.ndarray>(__params[1])
            alpha = <np.ndarray>(__params[2])
            d = <np.ndarray>(__params[3])
            base_pose = <Pose3>(__params[4])
            theta_bias = <np.ndarray>(__params[5])
            assert isinstance(a, np.ndarray) and a.ndim == 1
            assert isinstance(alpha, np.ndarray) and alpha.ndim == 1
            assert isinstance(d, np.ndarray) and d.ndim == 1
            assert isinstance(base_pose, Pose3)
            assert isinstance(theta_bias, np.ndarray) and theta_bias.ndim == 1
            a = a.astype(float, order='F', copy=False)
            alpha = alpha.astype(float, order='F', copy=False)
            d = d.astype(float, order='F', copy=False)
            theta_bias = theta_bias.astype(float, order='F', copy=False)
            self.CArm_ = shared_ptr[CArm](new CArm(dof, <VectorXd>(Map[VectorXd](a)), <VectorXd>(Map[VectorXd](alpha)), <VectorXd>(Map[VectorXd](d)), deref(base_pose.CPose3_), <VectorXd>(Map[VectorXd](theta_bias))))
        except (AssertionError, ValueError):
            pass
        if (self.CArm_.use_count()==0):
            raise TypeError('Arm construction failed!')

    @staticmethod
    cdef Arm cyCreateFromShared(const shared_ptr[CArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Arm return_value = Arm(cyCreateFromShared=True)
        return_value.CArm_ = other
        return return_value

    def a(self):
        cdef VectorXd ret = self.CArm_.get().a()
        return ndarray_copy(ret).squeeze()
    def alpha(self):
        cdef VectorXd ret = self.CArm_.get().alpha()
        return ndarray_copy(ret).squeeze()
    def base_pose(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CArm_.get().base_pose())
        return Pose3.cyCreateFromShared(ret)
    def d(self):
        cdef VectorXd ret = self.CArm_.get().d()
        return ndarray_copy(ret).squeeze()
    def dof(self):
        cdef size_t ret = self.CArm_.get().dof()
        return ret
    def forwardKinematicsPose(self, np.ndarray jp):
        jp = jp.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CArm_.get().forwardKinematicsPose(<VectorXd>(Map[VectorXd](jp)))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, np.ndarray jp):
        jp = jp.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CArm_.get().forwardKinematicsPosition(<VectorXd>(Map[VectorXd](jp)))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, np.ndarray jp, np.ndarray jv):
        jp = jp.astype(float, order='F', copy=False)
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CArm_.get().forwardKinematicsVel(<VectorXd>(Map[VectorXd](jp)), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)


cdef class Pose2MobileBase:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileBase_ = shared_ptr[CPose2MobileBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CPose2MobileBase_ = shared_ptr[CPose2MobileBase](new CPose2MobileBase())
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileBase_.use_count()==0):
            raise TypeError('Pose2MobileBase construction failed!')

    @staticmethod
    cdef Pose2MobileBase cyCreateFromShared(const shared_ptr[CPose2MobileBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileBase return_value = Pose2MobileBase(cyCreateFromShared=True)
        return_value.CPose2MobileBase_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPose2MobileBase_.get().dof()
        return ret
    def forwardKinematicsPose(self, Pose2 jp):
        cdef MatrixXd ret = self.CPose2MobileBase_.get().forwardKinematicsPose(deref(jp.CPose2_))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, Pose2 jp):
        cdef MatrixXd ret = self.CPose2MobileBase_.get().forwardKinematicsPosition(deref(jp.CPose2_))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, Pose2 jp, np.ndarray jv):
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPose2MobileBase_.get().forwardKinematicsVel(deref(jp.CPose2_), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPose2MobileBase_.get().nr_links()
        return ret


cdef class Pose2MobileArm:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileArm_ = shared_ptr[CPose2MobileArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm'], args, kwargs)
            arm = <Arm>(__params[0])
            assert isinstance(arm, Arm)
            self.CPose2MobileArm_ = shared_ptr[CPose2MobileArm](new CPose2MobileArm(deref(arm.CArm_)))
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['arm', 'base_T_arm'], args, kwargs)
            arm = <Arm>(__params[0])
            base_T_arm = <Pose3>(__params[1])
            assert isinstance(arm, Arm)
            assert isinstance(base_T_arm, Pose3)
            self.CPose2MobileArm_ = shared_ptr[CPose2MobileArm](new CPose2MobileArm(deref(arm.CArm_), deref(base_T_arm.CPose3_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileArm_.use_count()==0):
            raise TypeError('Pose2MobileArm construction failed!')

    @staticmethod
    cdef Pose2MobileArm cyCreateFromShared(const shared_ptr[CPose2MobileArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileArm return_value = Pose2MobileArm(cyCreateFromShared=True)
        return_value.CPose2MobileArm_ = other
        return return_value

    def arm(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CPose2MobileArm_.get().arm())
        return Arm.cyCreateFromShared(ret)
    def base_T_arm(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2MobileArm_.get().base_T_arm())
        return Pose3.cyCreateFromShared(ret)
    def dof(self):
        cdef size_t ret = self.CPose2MobileArm_.get().dof()
        return ret
    def forwardKinematicsPose(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2MobileArm_.get().forwardKinematicsPose(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2MobileArm_.get().forwardKinematicsPosition(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, Pose2Vector jp, np.ndarray jv):
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPose2MobileArm_.get().forwardKinematicsVel(deref(jp.CPose2Vector_), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPose2MobileArm_.get().nr_links()
        return ret


cdef class Pose2Mobile2Arms:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2Mobile2Arms_ = shared_ptr[CPose2Mobile2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm1', 'arm2'], args, kwargs)
            arm1 = <Arm>(__params[0])
            arm2 = <Arm>(__params[1])
            assert isinstance(arm1, Arm)
            assert isinstance(arm2, Arm)
            self.CPose2Mobile2Arms_ = shared_ptr[CPose2Mobile2Arms](new CPose2Mobile2Arms(deref(arm1.CArm_), deref(arm2.CArm_)))
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['arm1', 'arm2', 'base_T_arm1', 'base_T_arm2'], args, kwargs)
            arm1 = <Arm>(__params[0])
            arm2 = <Arm>(__params[1])
            base_T_arm1 = <Pose3>(__params[2])
            base_T_arm2 = <Pose3>(__params[3])
            assert isinstance(arm1, Arm)
            assert isinstance(arm2, Arm)
            assert isinstance(base_T_arm1, Pose3)
            assert isinstance(base_T_arm2, Pose3)
            self.CPose2Mobile2Arms_ = shared_ptr[CPose2Mobile2Arms](new CPose2Mobile2Arms(deref(arm1.CArm_), deref(arm2.CArm_), deref(base_T_arm1.CPose3_), deref(base_T_arm2.CPose3_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2Mobile2Arms_.use_count()==0):
            raise TypeError('Pose2Mobile2Arms construction failed!')

    @staticmethod
    cdef Pose2Mobile2Arms cyCreateFromShared(const shared_ptr[CPose2Mobile2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2Mobile2Arms return_value = Pose2Mobile2Arms(cyCreateFromShared=True)
        return_value.CPose2Mobile2Arms_ = other
        return return_value

    def arm1(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CPose2Mobile2Arms_.get().arm1())
        return Arm.cyCreateFromShared(ret)
    def arm2(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CPose2Mobile2Arms_.get().arm2())
        return Arm.cyCreateFromShared(ret)
    def base_T_arm1(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2Mobile2Arms_.get().base_T_arm1())
        return Pose3.cyCreateFromShared(ret)
    def base_T_arm2(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2Mobile2Arms_.get().base_T_arm2())
        return Pose3.cyCreateFromShared(ret)
    def dof(self):
        cdef size_t ret = self.CPose2Mobile2Arms_.get().dof()
        return ret
    def forwardKinematicsPose(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2Mobile2Arms_.get().forwardKinematicsPose(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2Mobile2Arms_.get().forwardKinematicsPosition(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, Pose2Vector jp, np.ndarray jv):
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPose2Mobile2Arms_.get().forwardKinematicsVel(deref(jp.CPose2Vector_), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPose2Mobile2Arms_.get().nr_links()
        return ret


cdef class Pose2MobileVetLinArm:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileVetLinArm_ = shared_ptr[CPose2MobileVetLinArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm'], args, kwargs)
            arm = <Arm>(__params[0])
            assert isinstance(arm, Arm)
            self.CPose2MobileVetLinArm_ = shared_ptr[CPose2MobileVetLinArm](new CPose2MobileVetLinArm(deref(arm.CArm_)))
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['arm', 'base_T_torso', 'torso_T_arm', 'reverse_linact'], args, kwargs)
            arm = <Arm>(__params[0])
            base_T_torso = <Pose3>(__params[1])
            torso_T_arm = <Pose3>(__params[2])
            reverse_linact = <bool>(__params[3])
            assert isinstance(arm, Arm)
            assert isinstance(base_T_torso, Pose3)
            assert isinstance(torso_T_arm, Pose3)
            self.CPose2MobileVetLinArm_ = shared_ptr[CPose2MobileVetLinArm](new CPose2MobileVetLinArm(deref(arm.CArm_), deref(base_T_torso.CPose3_), deref(torso_T_arm.CPose3_), reverse_linact))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileVetLinArm_.use_count()==0):
            raise TypeError('Pose2MobileVetLinArm construction failed!')

    @staticmethod
    cdef Pose2MobileVetLinArm cyCreateFromShared(const shared_ptr[CPose2MobileVetLinArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileVetLinArm return_value = Pose2MobileVetLinArm(cyCreateFromShared=True)
        return_value.CPose2MobileVetLinArm_ = other
        return return_value

    def arm(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CPose2MobileVetLinArm_.get().arm())
        return Arm.cyCreateFromShared(ret)
    def base_T_torso(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2MobileVetLinArm_.get().base_T_torso())
        return Pose3.cyCreateFromShared(ret)
    def dof(self):
        cdef size_t ret = self.CPose2MobileVetLinArm_.get().dof()
        return ret
    def forwardKinematicsPose(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2MobileVetLinArm_.get().forwardKinematicsPose(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2MobileVetLinArm_.get().forwardKinematicsPosition(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, Pose2Vector jp, np.ndarray jv):
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPose2MobileVetLinArm_.get().forwardKinematicsVel(deref(jp.CPose2Vector_), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPose2MobileVetLinArm_.get().nr_links()
        return ret
    def reverse_linact(self):
        cdef bool ret = self.CPose2MobileVetLinArm_.get().reverse_linact()
        return ret
    def torso_T_arm(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2MobileVetLinArm_.get().torso_T_arm())
        return Pose3.cyCreateFromShared(ret)


cdef class Pose2MobileVetLin2Arms:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileVetLin2Arms_ = shared_ptr[CPose2MobileVetLin2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm1', 'arm2'], args, kwargs)
            arm1 = <Arm>(__params[0])
            arm2 = <Arm>(__params[1])
            assert isinstance(arm1, Arm)
            assert isinstance(arm2, Arm)
            self.CPose2MobileVetLin2Arms_ = shared_ptr[CPose2MobileVetLin2Arms](new CPose2MobileVetLin2Arms(deref(arm1.CArm_), deref(arm2.CArm_)))
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['arm1', 'arm2', 'base_T_torso', 'torso_T_arm1', 'torso_T_arm2', 'reverse_linact'], args, kwargs)
            arm1 = <Arm>(__params[0])
            arm2 = <Arm>(__params[1])
            base_T_torso = <Pose3>(__params[2])
            torso_T_arm1 = <Pose3>(__params[3])
            torso_T_arm2 = <Pose3>(__params[4])
            reverse_linact = <bool>(__params[5])
            assert isinstance(arm1, Arm)
            assert isinstance(arm2, Arm)
            assert isinstance(base_T_torso, Pose3)
            assert isinstance(torso_T_arm1, Pose3)
            assert isinstance(torso_T_arm2, Pose3)
            self.CPose2MobileVetLin2Arms_ = shared_ptr[CPose2MobileVetLin2Arms](new CPose2MobileVetLin2Arms(deref(arm1.CArm_), deref(arm2.CArm_), deref(base_T_torso.CPose3_), deref(torso_T_arm1.CPose3_), deref(torso_T_arm2.CPose3_), reverse_linact))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileVetLin2Arms_.use_count()==0):
            raise TypeError('Pose2MobileVetLin2Arms construction failed!')

    @staticmethod
    cdef Pose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CPose2MobileVetLin2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileVetLin2Arms return_value = Pose2MobileVetLin2Arms(cyCreateFromShared=True)
        return_value.CPose2MobileVetLin2Arms_ = other
        return return_value

    def arm1(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CPose2MobileVetLin2Arms_.get().arm1())
        return Arm.cyCreateFromShared(ret)
    def arm2(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CPose2MobileVetLin2Arms_.get().arm2())
        return Arm.cyCreateFromShared(ret)
    def base_T_torso(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2MobileVetLin2Arms_.get().base_T_torso())
        return Pose3.cyCreateFromShared(ret)
    def dof(self):
        cdef size_t ret = self.CPose2MobileVetLin2Arms_.get().dof()
        return ret
    def forwardKinematicsPose(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2MobileVetLin2Arms_.get().forwardKinematicsPose(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, Pose2Vector jp):
        cdef MatrixXd ret = self.CPose2MobileVetLin2Arms_.get().forwardKinematicsPosition(deref(jp.CPose2Vector_))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, Pose2Vector jp, np.ndarray jv):
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPose2MobileVetLin2Arms_.get().forwardKinematicsVel(deref(jp.CPose2Vector_), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPose2MobileVetLin2Arms_.get().nr_links()
        return ret
    def reverse_linact(self):
        cdef bool ret = self.CPose2MobileVetLin2Arms_.get().reverse_linact()
        return ret
    def torso_T_arm1(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2MobileVetLin2Arms_.get().torso_T_arm1())
        return Pose3.cyCreateFromShared(ret)
    def torso_T_arm2(self):
        cdef shared_ptr[CPose3] ret = make_shared[CPose3](self.CPose2MobileVetLin2Arms_.get().torso_T_arm2())
        return Pose3.cyCreateFromShared(ret)


cdef class PointRobot:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPointRobot_ = shared_ptr[CPointRobot]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['dof', 'nr_links'], args, kwargs)
            dof = <size_t>(__params[0])
            nr_links = <size_t>(__params[1])
            self.CPointRobot_ = shared_ptr[CPointRobot](new CPointRobot(dof, nr_links))
        except (AssertionError, ValueError):
            pass
        if (self.CPointRobot_.use_count()==0):
            raise TypeError('PointRobot construction failed!')

    @staticmethod
    cdef PointRobot cyCreateFromShared(const shared_ptr[CPointRobot]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PointRobot return_value = PointRobot(cyCreateFromShared=True)
        return_value.CPointRobot_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPointRobot_.get().dof()
        return ret
    def forwardKinematicsPose(self, np.ndarray jp):
        jp = jp.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobot_.get().forwardKinematicsPose(<VectorXd>(Map[VectorXd](jp)))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, np.ndarray jp):
        jp = jp.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobot_.get().forwardKinematicsPosition(<VectorXd>(Map[VectorXd](jp)))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, np.ndarray jp, np.ndarray jv):
        jp = jp.astype(float, order='F', copy=False)
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobot_.get().forwardKinematicsVel(<VectorXd>(Map[VectorXd](jp)), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPointRobot_.get().nr_links()
        return ret


cdef class BodySphere:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CBodySphere_ = shared_ptr[CBodySphere]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['id', 'r', 'c'], args, kwargs)
            id = <size_t>(__params[0])
            r = <double>(__params[1])
            c = <Point3>(__params[2])
            assert isinstance(c, Point3)
            self.CBodySphere_ = shared_ptr[CBodySphere](new CBodySphere(id, r, deref(c.CPoint3_)))
        except (AssertionError, ValueError):
            pass
        if (self.CBodySphere_.use_count()==0):
            raise TypeError('BodySphere construction failed!')

    @staticmethod
    cdef BodySphere cyCreateFromShared(const shared_ptr[CBodySphere]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef BodySphere return_value = BodySphere(cyCreateFromShared=True)
        return_value.CBodySphere_ = other
        return return_value



cdef class BodySphereVector:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CBodySphereVector_ = shared_ptr[CBodySphereVector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CBodySphereVector_ = shared_ptr[CBodySphereVector](new CBodySphereVector())
        except (AssertionError, ValueError):
            pass
        if (self.CBodySphereVector_.use_count()==0):
            raise TypeError('BodySphereVector construction failed!')

    @staticmethod
    cdef BodySphereVector cyCreateFromShared(const shared_ptr[CBodySphereVector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef BodySphereVector return_value = BodySphereVector(cyCreateFromShared=True)
        return_value.CBodySphereVector_ = other
        return return_value

    def push_back(self, BodySphere sphere):
        self.CBodySphereVector_.get().push_back(deref(sphere.CBodySphere_))


cdef class ArmModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CArmModel_ = shared_ptr[CArmModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm', 'spheres'], args, kwargs)
            arm = <Arm>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(arm, Arm)
            assert isinstance(spheres, BodySphereVector)
            self.CArmModel_ = shared_ptr[CArmModel](new CArmModel(deref(arm.CArm_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CArmModel_.use_count()==0):
            raise TypeError('ArmModel construction failed!')

    @staticmethod
    cdef ArmModel cyCreateFromShared(const shared_ptr[CArmModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ArmModel return_value = ArmModel(cyCreateFromShared=True)
        return_value.CArmModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CArmModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CArm] ret = make_shared[CArm](self.CArmModel_.get().fk_model())
        return Arm.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CArmModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, np.ndarray conf):
        conf = conf.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CArmModel_.get().sphereCentersMat(<VectorXd>(Map[VectorXd](conf)))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CArmModel_.get().sphere_radius(i)
        return ret


cdef class Pose2MobileBaseModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileBaseModel_ = shared_ptr[CPose2MobileBaseModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['r', 'spheres'], args, kwargs)
            r = <Pose2MobileBase>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(r, Pose2MobileBase)
            assert isinstance(spheres, BodySphereVector)
            self.CPose2MobileBaseModel_ = shared_ptr[CPose2MobileBaseModel](new CPose2MobileBaseModel(deref(r.CPose2MobileBase_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileBaseModel_.use_count()==0):
            raise TypeError('Pose2MobileBaseModel construction failed!')

    @staticmethod
    cdef Pose2MobileBaseModel cyCreateFromShared(const shared_ptr[CPose2MobileBaseModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileBaseModel return_value = Pose2MobileBaseModel(cyCreateFromShared=True)
        return_value.CPose2MobileBaseModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPose2MobileBaseModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPose2MobileBase] ret = make_shared[CPose2MobileBase](self.CPose2MobileBaseModel_.get().fk_model())
        return Pose2MobileBase.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPose2MobileBaseModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, Pose2 conf):
        cdef MatrixXd ret = self.CPose2MobileBaseModel_.get().sphereCentersMat(deref(conf.CPose2_))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPose2MobileBaseModel_.get().sphere_radius(i)
        return ret


cdef class Pose2MobileArmModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileArmModel_ = shared_ptr[CPose2MobileArmModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['r', 'spheres'], args, kwargs)
            r = <Pose2MobileArm>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(r, Pose2MobileArm)
            assert isinstance(spheres, BodySphereVector)
            self.CPose2MobileArmModel_ = shared_ptr[CPose2MobileArmModel](new CPose2MobileArmModel(deref(r.CPose2MobileArm_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileArmModel_.use_count()==0):
            raise TypeError('Pose2MobileArmModel construction failed!')

    @staticmethod
    cdef Pose2MobileArmModel cyCreateFromShared(const shared_ptr[CPose2MobileArmModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileArmModel return_value = Pose2MobileArmModel(cyCreateFromShared=True)
        return_value.CPose2MobileArmModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPose2MobileArmModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPose2MobileArm] ret = make_shared[CPose2MobileArm](self.CPose2MobileArmModel_.get().fk_model())
        return Pose2MobileArm.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPose2MobileArmModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, Pose2Vector conf):
        cdef MatrixXd ret = self.CPose2MobileArmModel_.get().sphereCentersMat(deref(conf.CPose2Vector_))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPose2MobileArmModel_.get().sphere_radius(i)
        return ret


cdef class Pose2Mobile2ArmsModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2Mobile2ArmsModel_ = shared_ptr[CPose2Mobile2ArmsModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['r', 'spheres'], args, kwargs)
            r = <Pose2Mobile2Arms>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(r, Pose2Mobile2Arms)
            assert isinstance(spheres, BodySphereVector)
            self.CPose2Mobile2ArmsModel_ = shared_ptr[CPose2Mobile2ArmsModel](new CPose2Mobile2ArmsModel(deref(r.CPose2Mobile2Arms_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2Mobile2ArmsModel_.use_count()==0):
            raise TypeError('Pose2Mobile2ArmsModel construction failed!')

    @staticmethod
    cdef Pose2Mobile2ArmsModel cyCreateFromShared(const shared_ptr[CPose2Mobile2ArmsModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2Mobile2ArmsModel return_value = Pose2Mobile2ArmsModel(cyCreateFromShared=True)
        return_value.CPose2Mobile2ArmsModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPose2Mobile2ArmsModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPose2Mobile2Arms] ret = make_shared[CPose2Mobile2Arms](self.CPose2Mobile2ArmsModel_.get().fk_model())
        return Pose2Mobile2Arms.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPose2Mobile2ArmsModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, Pose2Vector conf):
        cdef MatrixXd ret = self.CPose2Mobile2ArmsModel_.get().sphereCentersMat(deref(conf.CPose2Vector_))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPose2Mobile2ArmsModel_.get().sphere_radius(i)
        return ret


cdef class Pose2MobileVetLinArmModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileVetLinArmModel_ = shared_ptr[CPose2MobileVetLinArmModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['r', 'spheres'], args, kwargs)
            r = <Pose2MobileVetLinArm>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(r, Pose2MobileVetLinArm)
            assert isinstance(spheres, BodySphereVector)
            self.CPose2MobileVetLinArmModel_ = shared_ptr[CPose2MobileVetLinArmModel](new CPose2MobileVetLinArmModel(deref(r.CPose2MobileVetLinArm_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileVetLinArmModel_.use_count()==0):
            raise TypeError('Pose2MobileVetLinArmModel construction failed!')

    @staticmethod
    cdef Pose2MobileVetLinArmModel cyCreateFromShared(const shared_ptr[CPose2MobileVetLinArmModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileVetLinArmModel return_value = Pose2MobileVetLinArmModel(cyCreateFromShared=True)
        return_value.CPose2MobileVetLinArmModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPose2MobileVetLinArmModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPose2MobileVetLinArm] ret = make_shared[CPose2MobileVetLinArm](self.CPose2MobileVetLinArmModel_.get().fk_model())
        return Pose2MobileVetLinArm.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPose2MobileVetLinArmModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, Pose2Vector conf):
        cdef MatrixXd ret = self.CPose2MobileVetLinArmModel_.get().sphereCentersMat(deref(conf.CPose2Vector_))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPose2MobileVetLinArmModel_.get().sphere_radius(i)
        return ret


cdef class Pose2MobileVetLin2ArmsModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPose2MobileVetLin2ArmsModel_ = shared_ptr[CPose2MobileVetLin2ArmsModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['r', 'spheres'], args, kwargs)
            r = <Pose2MobileVetLin2Arms>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(r, Pose2MobileVetLin2Arms)
            assert isinstance(spheres, BodySphereVector)
            self.CPose2MobileVetLin2ArmsModel_ = shared_ptr[CPose2MobileVetLin2ArmsModel](new CPose2MobileVetLin2ArmsModel(deref(r.CPose2MobileVetLin2Arms_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPose2MobileVetLin2ArmsModel_.use_count()==0):
            raise TypeError('Pose2MobileVetLin2ArmsModel construction failed!')

    @staticmethod
    cdef Pose2MobileVetLin2ArmsModel cyCreateFromShared(const shared_ptr[CPose2MobileVetLin2ArmsModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Pose2MobileVetLin2ArmsModel return_value = Pose2MobileVetLin2ArmsModel(cyCreateFromShared=True)
        return_value.CPose2MobileVetLin2ArmsModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPose2MobileVetLin2ArmsModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPose2MobileVetLin2Arms] ret = make_shared[CPose2MobileVetLin2Arms](self.CPose2MobileVetLin2ArmsModel_.get().fk_model())
        return Pose2MobileVetLin2Arms.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPose2MobileVetLin2ArmsModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, Pose2Vector conf):
        cdef MatrixXd ret = self.CPose2MobileVetLin2ArmsModel_.get().sphereCentersMat(deref(conf.CPose2Vector_))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPose2MobileVetLin2ArmsModel_.get().sphere_radius(i)
        return ret


cdef class PointRobotModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPointRobotModel_ = shared_ptr[CPointRobotModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pR', 'spheres'], args, kwargs)
            pR = <PointRobot>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(pR, PointRobot)
            assert isinstance(spheres, BodySphereVector)
            self.CPointRobotModel_ = shared_ptr[CPointRobotModel](new CPointRobotModel(deref(pR.CPointRobot_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPointRobotModel_.use_count()==0):
            raise TypeError('PointRobotModel construction failed!')

    @staticmethod
    cdef PointRobotModel cyCreateFromShared(const shared_ptr[CPointRobotModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PointRobotModel return_value = PointRobotModel(cyCreateFromShared=True)
        return_value.CPointRobotModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPointRobotModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPointRobot] ret = make_shared[CPointRobot](self.CPointRobotModel_.get().fk_model())
        return PointRobot.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPointRobotModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, np.ndarray conf):
        conf = conf.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobotModel_.get().sphereCentersMat(<VectorXd>(Map[VectorXd](conf)))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPointRobotModel_.get().sphere_radius(i)
        return ret


cdef class GoalFactorArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGoalFactorArm_ = shared_ptr[CGoalFactorArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'cost_model', 'arm', 'dest_point'], args, kwargs)
            poseKey = <size_t>(__params[0])
            cost_model = <noiseModel_Base>(__params[1])
            arm = <Arm>(__params[2])
            dest_point = <Point3>(__params[3])
            assert isinstance(cost_model, noiseModel_Base)
            assert isinstance(arm, Arm)
            assert isinstance(dest_point, Point3)
            self.CGoalFactorArm_ = shared_ptr[CGoalFactorArm](new CGoalFactorArm(poseKey, cost_model.CnoiseModel_Base_, deref(arm.CArm_), deref(dest_point.CPoint3_)))
        except (AssertionError, ValueError):
            pass
        if (self.CGoalFactorArm_.use_count()==0):
            raise TypeError('GoalFactorArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGoalFactorArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGoalFactorArm_)

    @staticmethod
    cdef GoalFactorArm cyCreateFromShared(const shared_ptr[CGoalFactorArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GoalFactorArm return_value = GoalFactorArm(cyCreateFromShared=True)
        return_value.CGoalFactorArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_GoalFactorArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GoalFactorArm.cyCreateFromShared(<shared_ptr[CGoalFactorArm]>dynamic_pointer_cast[CGoalFactorArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GoalFactorArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return GoalFactorArm.cyCreateFromShared(<shared_ptr[CGoalFactorArm]>dynamic_pointer_cast[CGoalFactorArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class JointLimitFactorVector(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CJointLimitFactorVector_ = shared_ptr[CJointLimitFactorVector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['key', 'cost_model', 'down_limit', 'up_limit', 'limit_thresh'], args, kwargs)
            key = <size_t>(__params[0])
            cost_model = <noiseModel_Base>(__params[1])
            down_limit = <np.ndarray>(__params[2])
            up_limit = <np.ndarray>(__params[3])
            limit_thresh = <np.ndarray>(__params[4])
            assert isinstance(cost_model, noiseModel_Base)
            assert isinstance(down_limit, np.ndarray) and down_limit.ndim == 1
            assert isinstance(up_limit, np.ndarray) and up_limit.ndim == 1
            assert isinstance(limit_thresh, np.ndarray) and limit_thresh.ndim == 1
            down_limit = down_limit.astype(float, order='F', copy=False)
            up_limit = up_limit.astype(float, order='F', copy=False)
            limit_thresh = limit_thresh.astype(float, order='F', copy=False)
            self.CJointLimitFactorVector_ = shared_ptr[CJointLimitFactorVector](new CJointLimitFactorVector(key, cost_model.CnoiseModel_Base_, <VectorXd>(Map[VectorXd](down_limit)), <VectorXd>(Map[VectorXd](up_limit)), <VectorXd>(Map[VectorXd](limit_thresh))))
        except (AssertionError, ValueError):
            pass
        if (self.CJointLimitFactorVector_.use_count()==0):
            raise TypeError('JointLimitFactorVector construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CJointLimitFactorVector_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CJointLimitFactorVector_)

    @staticmethod
    cdef JointLimitFactorVector cyCreateFromShared(const shared_ptr[CJointLimitFactorVector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef JointLimitFactorVector return_value = JointLimitFactorVector(cyCreateFromShared=True)
        return_value.CJointLimitFactorVector_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_JointLimitFactorVector_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return JointLimitFactorVector.cyCreateFromShared(<shared_ptr[CJointLimitFactorVector]>dynamic_pointer_cast[CJointLimitFactorVector,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_JointLimitFactorVector_NonlinearFactor(NonlinearFactor parent):
    try:
        return JointLimitFactorVector.cyCreateFromShared(<shared_ptr[CJointLimitFactorVector]>dynamic_pointer_cast[CJointLimitFactorVector,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class VelocityLimitFactorVector(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CVelocityLimitFactorVector_ = shared_ptr[CVelocityLimitFactorVector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['key', 'cost_model', 'vel_limit', 'limit_thresh'], args, kwargs)
            key = <size_t>(__params[0])
            cost_model = <noiseModel_Base>(__params[1])
            vel_limit = <np.ndarray>(__params[2])
            limit_thresh = <np.ndarray>(__params[3])
            assert isinstance(cost_model, noiseModel_Base)
            assert isinstance(vel_limit, np.ndarray) and vel_limit.ndim == 1
            assert isinstance(limit_thresh, np.ndarray) and limit_thresh.ndim == 1
            vel_limit = vel_limit.astype(float, order='F', copy=False)
            limit_thresh = limit_thresh.astype(float, order='F', copy=False)
            self.CVelocityLimitFactorVector_ = shared_ptr[CVelocityLimitFactorVector](new CVelocityLimitFactorVector(key, cost_model.CnoiseModel_Base_, <VectorXd>(Map[VectorXd](vel_limit)), <VectorXd>(Map[VectorXd](limit_thresh))))
        except (AssertionError, ValueError):
            pass
        if (self.CVelocityLimitFactorVector_.use_count()==0):
            raise TypeError('VelocityLimitFactorVector construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CVelocityLimitFactorVector_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CVelocityLimitFactorVector_)

    @staticmethod
    cdef VelocityLimitFactorVector cyCreateFromShared(const shared_ptr[CVelocityLimitFactorVector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef VelocityLimitFactorVector return_value = VelocityLimitFactorVector(cyCreateFromShared=True)
        return_value.CVelocityLimitFactorVector_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_VelocityLimitFactorVector_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return VelocityLimitFactorVector.cyCreateFromShared(<shared_ptr[CVelocityLimitFactorVector]>dynamic_pointer_cast[CVelocityLimitFactorVector,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_VelocityLimitFactorVector_NonlinearFactor(NonlinearFactor parent):
    try:
        return VelocityLimitFactorVector.cyCreateFromShared(<shared_ptr[CVelocityLimitFactorVector]>dynamic_pointer_cast[CVelocityLimitFactorVector,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class GaussianPriorWorkspacePositionArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianPriorWorkspacePositionArm_ = shared_ptr[CGaussianPriorWorkspacePositionArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'arm', 'joint', 'des_position', 'cost_model'], args, kwargs)
            poseKey = <size_t>(__params[0])
            arm = <ArmModel>(__params[1])
            joint = <int>(__params[2])
            des_position = <Point3>(__params[3])
            cost_model = <noiseModel_Base>(__params[4])
            assert isinstance(arm, ArmModel)
            assert isinstance(des_position, Point3)
            assert isinstance(cost_model, noiseModel_Base)
            self.CGaussianPriorWorkspacePositionArm_ = shared_ptr[CGaussianPriorWorkspacePositionArm](new CGaussianPriorWorkspacePositionArm(poseKey, deref(arm.CArmModel_), joint, deref(des_position.CPoint3_), cost_model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianPriorWorkspacePositionArm_.use_count()==0):
            raise TypeError('GaussianPriorWorkspacePositionArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGaussianPriorWorkspacePositionArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGaussianPriorWorkspacePositionArm_)

    @staticmethod
    cdef GaussianPriorWorkspacePositionArm cyCreateFromShared(const shared_ptr[CGaussianPriorWorkspacePositionArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianPriorWorkspacePositionArm return_value = GaussianPriorWorkspacePositionArm(cyCreateFromShared=True)
        return_value.CGaussianPriorWorkspacePositionArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_GaussianPriorWorkspacePositionArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GaussianPriorWorkspacePositionArm.cyCreateFromShared(<shared_ptr[CGaussianPriorWorkspacePositionArm]>dynamic_pointer_cast[CGaussianPriorWorkspacePositionArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GaussianPriorWorkspacePositionArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return GaussianPriorWorkspacePositionArm.cyCreateFromShared(<shared_ptr[CGaussianPriorWorkspacePositionArm]>dynamic_pointer_cast[CGaussianPriorWorkspacePositionArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class GaussianPriorWorkspaceOrientationArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianPriorWorkspaceOrientationArm_ = shared_ptr[CGaussianPriorWorkspaceOrientationArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'arm', 'joint', 'des_orientation', 'cost_model'], args, kwargs)
            poseKey = <size_t>(__params[0])
            arm = <ArmModel>(__params[1])
            joint = <int>(__params[2])
            des_orientation = <Rot3>(__params[3])
            cost_model = <noiseModel_Base>(__params[4])
            assert isinstance(arm, ArmModel)
            assert isinstance(des_orientation, Rot3)
            assert isinstance(cost_model, noiseModel_Base)
            self.CGaussianPriorWorkspaceOrientationArm_ = shared_ptr[CGaussianPriorWorkspaceOrientationArm](new CGaussianPriorWorkspaceOrientationArm(poseKey, deref(arm.CArmModel_), joint, deref(des_orientation.CRot3_), cost_model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianPriorWorkspaceOrientationArm_.use_count()==0):
            raise TypeError('GaussianPriorWorkspaceOrientationArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGaussianPriorWorkspaceOrientationArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGaussianPriorWorkspaceOrientationArm_)

    @staticmethod
    cdef GaussianPriorWorkspaceOrientationArm cyCreateFromShared(const shared_ptr[CGaussianPriorWorkspaceOrientationArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianPriorWorkspaceOrientationArm return_value = GaussianPriorWorkspaceOrientationArm(cyCreateFromShared=True)
        return_value.CGaussianPriorWorkspaceOrientationArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_GaussianPriorWorkspaceOrientationArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GaussianPriorWorkspaceOrientationArm.cyCreateFromShared(<shared_ptr[CGaussianPriorWorkspaceOrientationArm]>dynamic_pointer_cast[CGaussianPriorWorkspaceOrientationArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GaussianPriorWorkspaceOrientationArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return GaussianPriorWorkspaceOrientationArm.cyCreateFromShared(<shared_ptr[CGaussianPriorWorkspaceOrientationArm]>dynamic_pointer_cast[CGaussianPriorWorkspaceOrientationArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class GaussianPriorWorkspacePoseArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CGaussianPriorWorkspacePoseArm_ = shared_ptr[CGaussianPriorWorkspacePoseArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'arm', 'joint', 'des_pose', 'cost_model'], args, kwargs)
            poseKey = <size_t>(__params[0])
            arm = <ArmModel>(__params[1])
            joint = <int>(__params[2])
            des_pose = <Pose3>(__params[3])
            cost_model = <noiseModel_Base>(__params[4])
            assert isinstance(arm, ArmModel)
            assert isinstance(des_pose, Pose3)
            assert isinstance(cost_model, noiseModel_Base)
            self.CGaussianPriorWorkspacePoseArm_ = shared_ptr[CGaussianPriorWorkspacePoseArm](new CGaussianPriorWorkspacePoseArm(poseKey, deref(arm.CArmModel_), joint, deref(des_pose.CPose3_), cost_model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CGaussianPriorWorkspacePoseArm_.use_count()==0):
            raise TypeError('GaussianPriorWorkspacePoseArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CGaussianPriorWorkspacePoseArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CGaussianPriorWorkspacePoseArm_)

    @staticmethod
    cdef GaussianPriorWorkspacePoseArm cyCreateFromShared(const shared_ptr[CGaussianPriorWorkspacePoseArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef GaussianPriorWorkspacePoseArm return_value = GaussianPriorWorkspacePoseArm(cyCreateFromShared=True)
        return_value.CGaussianPriorWorkspacePoseArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_GaussianPriorWorkspacePoseArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return GaussianPriorWorkspacePoseArm.cyCreateFromShared(<shared_ptr[CGaussianPriorWorkspacePoseArm]>dynamic_pointer_cast[CGaussianPriorWorkspacePoseArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_GaussianPriorWorkspacePoseArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return GaussianPriorWorkspacePoseArm.cyCreateFromShared(<shared_ptr[CGaussianPriorWorkspacePoseArm]>dynamic_pointer_cast[CGaussianPriorWorkspacePoseArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class VehicleDynamicsFactorPose2(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CVehicleDynamicsFactorPose2_ = shared_ptr[CVehicleDynamicsFactorPose2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'velKey', 'cost_sigma'], args, kwargs)
            poseKey = <size_t>(__params[0])
            velKey = <size_t>(__params[1])
            cost_sigma = <double>(__params[2])
            self.CVehicleDynamicsFactorPose2_ = shared_ptr[CVehicleDynamicsFactorPose2](new CVehicleDynamicsFactorPose2(poseKey, velKey, cost_sigma))
        except (AssertionError, ValueError):
            pass
        if (self.CVehicleDynamicsFactorPose2_.use_count()==0):
            raise TypeError('VehicleDynamicsFactorPose2 construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CVehicleDynamicsFactorPose2_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CVehicleDynamicsFactorPose2_)

    @staticmethod
    cdef VehicleDynamicsFactorPose2 cyCreateFromShared(const shared_ptr[CVehicleDynamicsFactorPose2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef VehicleDynamicsFactorPose2 return_value = VehicleDynamicsFactorPose2(cyCreateFromShared=True)
        return_value.CVehicleDynamicsFactorPose2_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_VehicleDynamicsFactorPose2_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return VehicleDynamicsFactorPose2.cyCreateFromShared(<shared_ptr[CVehicleDynamicsFactorPose2]>dynamic_pointer_cast[CVehicleDynamicsFactorPose2,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_VehicleDynamicsFactorPose2_NonlinearFactor(NonlinearFactor parent):
    try:
        return VehicleDynamicsFactorPose2.cyCreateFromShared(<shared_ptr[CVehicleDynamicsFactorPose2]>dynamic_pointer_cast[CVehicleDynamicsFactorPose2,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class VehicleDynamicsFactorPose2Vector(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CVehicleDynamicsFactorPose2Vector_ = shared_ptr[CVehicleDynamicsFactorPose2Vector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'velKey', 'cost_sigma'], args, kwargs)
            poseKey = <size_t>(__params[0])
            velKey = <size_t>(__params[1])
            cost_sigma = <double>(__params[2])
            self.CVehicleDynamicsFactorPose2Vector_ = shared_ptr[CVehicleDynamicsFactorPose2Vector](new CVehicleDynamicsFactorPose2Vector(poseKey, velKey, cost_sigma))
        except (AssertionError, ValueError):
            pass
        if (self.CVehicleDynamicsFactorPose2Vector_.use_count()==0):
            raise TypeError('VehicleDynamicsFactorPose2Vector construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CVehicleDynamicsFactorPose2Vector_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CVehicleDynamicsFactorPose2Vector_)

    @staticmethod
    cdef VehicleDynamicsFactorPose2Vector cyCreateFromShared(const shared_ptr[CVehicleDynamicsFactorPose2Vector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef VehicleDynamicsFactorPose2Vector return_value = VehicleDynamicsFactorPose2Vector(cyCreateFromShared=True)
        return_value.CVehicleDynamicsFactorPose2Vector_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_VehicleDynamicsFactorPose2Vector_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return VehicleDynamicsFactorPose2Vector.cyCreateFromShared(<shared_ptr[CVehicleDynamicsFactorPose2Vector]>dynamic_pointer_cast[CVehicleDynamicsFactorPose2Vector,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_VehicleDynamicsFactorPose2Vector_NonlinearFactor(NonlinearFactor parent):
    try:
        return VehicleDynamicsFactorPose2Vector.cyCreateFromShared(<shared_ptr[CVehicleDynamicsFactorPose2Vector]>dynamic_pointer_cast[CVehicleDynamicsFactorPose2Vector,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class VehicleDynamicsFactorVector(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CVehicleDynamicsFactorVector_ = shared_ptr[CVehicleDynamicsFactorVector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'velKey', 'cost_sigma'], args, kwargs)
            poseKey = <size_t>(__params[0])
            velKey = <size_t>(__params[1])
            cost_sigma = <double>(__params[2])
            self.CVehicleDynamicsFactorVector_ = shared_ptr[CVehicleDynamicsFactorVector](new CVehicleDynamicsFactorVector(poseKey, velKey, cost_sigma))
        except (AssertionError, ValueError):
            pass
        if (self.CVehicleDynamicsFactorVector_.use_count()==0):
            raise TypeError('VehicleDynamicsFactorVector construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CVehicleDynamicsFactorVector_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CVehicleDynamicsFactorVector_)

    @staticmethod
    cdef VehicleDynamicsFactorVector cyCreateFromShared(const shared_ptr[CVehicleDynamicsFactorVector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef VehicleDynamicsFactorVector return_value = VehicleDynamicsFactorVector(cyCreateFromShared=True)
        return_value.CVehicleDynamicsFactorVector_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_VehicleDynamicsFactorVector_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return VehicleDynamicsFactorVector.cyCreateFromShared(<shared_ptr[CVehicleDynamicsFactorVector]>dynamic_pointer_cast[CVehicleDynamicsFactorVector,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_VehicleDynamicsFactorVector_NonlinearFactor(NonlinearFactor parent):
    try:
        return VehicleDynamicsFactorVector.cyCreateFromShared(<shared_ptr[CVehicleDynamicsFactorVector]>dynamic_pointer_cast[CVehicleDynamicsFactorVector,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class SignedDistanceField:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CSignedDistanceField_ = shared_ptr[CSignedDistanceField]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CSignedDistanceField_ = shared_ptr[CSignedDistanceField](new CSignedDistanceField())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['origin', 'cell_size', 'field_rows', 'field_cols', 'field_z'], args, kwargs)
            origin = <Point3>(__params[0])
            cell_size = <double>(__params[1])
            field_rows = <size_t>(__params[2])
            field_cols = <size_t>(__params[3])
            field_z = <size_t>(__params[4])
            assert isinstance(origin, Point3)
            self.CSignedDistanceField_ = shared_ptr[CSignedDistanceField](new CSignedDistanceField(deref(origin.CPoint3_), cell_size, field_rows, field_cols, field_z))
        except (AssertionError, ValueError):
            pass
        if (self.CSignedDistanceField_.use_count()==0):
            raise TypeError('SignedDistanceField construction failed!')

    @staticmethod
    cdef SignedDistanceField cyCreateFromShared(const shared_ptr[CSignedDistanceField]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef SignedDistanceField return_value = SignedDistanceField(cyCreateFromShared=True)
        return_value.CSignedDistanceField_ = other
        return return_value

    def getSignedDistance(self, Point3 point):
        cdef double ret = self.CSignedDistanceField_.get().getSignedDistance(deref(point.CPoint3_))
        return ret
    def initFieldData(self, size_t z_idx, np.ndarray field_layer):
        field_layer = field_layer.astype(float, order='F', copy=False)
        self.CSignedDistanceField_.get().initFieldData(z_idx, <MatrixXd>(Map[MatrixXd](field_layer)))
    def loadSDF(self, string filename):
        self.CSignedDistanceField_.get().loadSDF(filename)
    def __str__(self):
        strBuf = RedirectCout()
        self.print_('')
        return strBuf.str()
    def print_(self, string s):
        self.CSignedDistanceField_.get().print_(s)
    def saveSDF(self, string filename):
        self.CSignedDistanceField_.get().saveSDF(filename)


cdef class PlanarSDF:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPlanarSDF_ = shared_ptr[CPlanarSDF]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['origin', 'cell_size', 'data'], args, kwargs)
            origin = <Point2>(__params[0])
            cell_size = <double>(__params[1])
            data = <np.ndarray>(__params[2])
            assert isinstance(origin, Point2)
            assert isinstance(data, np.ndarray) and data.ndim == 2
            data = data.astype(float, order='F', copy=False)
            self.CPlanarSDF_ = shared_ptr[CPlanarSDF](new CPlanarSDF(deref(origin.CPoint2_), cell_size, <MatrixXd>(Map[MatrixXd](data))))
        except (AssertionError, ValueError):
            pass
        if (self.CPlanarSDF_.use_count()==0):
            raise TypeError('PlanarSDF construction failed!')

    @staticmethod
    cdef PlanarSDF cyCreateFromShared(const shared_ptr[CPlanarSDF]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PlanarSDF return_value = PlanarSDF(cyCreateFromShared=True)
        return_value.CPlanarSDF_ = other
        return return_value

    def getSignedDistance(self, Point2 point):
        cdef double ret = self.CPlanarSDF_.get().getSignedDistance(deref(point.CPoint2_))
        return ret
    def __str__(self):
        strBuf = RedirectCout()
        self.print_('')
        return strBuf.str()
    def print_(self, string s):
        self.CPlanarSDF_.get().print_(s)


cdef class ObstacleSDFFactorArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorArm_ = shared_ptr[CObstacleSDFFactorArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'arm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            poseKey = <size_t>(__params[0])
            arm = <ArmModel>(__params[1])
            sdf = <SignedDistanceField>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(arm, ArmModel)
            assert isinstance(sdf, SignedDistanceField)
            self.CObstacleSDFFactorArm_ = shared_ptr[CObstacleSDFFactorArm](new CObstacleSDFFactorArm(poseKey, deref(arm.CArmModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorArm_.use_count()==0):
            raise TypeError('ObstacleSDFFactorArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorArm_)

    @staticmethod
    cdef ObstacleSDFFactorArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorArm return_value = ObstacleSDFFactorArm(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, np.ndarray pose):
        pose = pose.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CObstacleSDFFactorArm_.get().evaluateError(<VectorXd>(Map[VectorXd](pose)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstacleSDFFactorArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorArm]>dynamic_pointer_cast[CObstacleSDFFactorArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorArm]>dynamic_pointer_cast[CObstacleSDFFactorArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorGPArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorGPArm_ = shared_ptr[CObstacleSDFFactorGPArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1Key', 'vel1Key', 'pose2Key', 'vel2Key', 'arm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1Key = <size_t>(__params[0])
            vel1Key = <size_t>(__params[1])
            pose2Key = <size_t>(__params[2])
            vel2Key = <size_t>(__params[3])
            arm = <ArmModel>(__params[4])
            sdf = <SignedDistanceField>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(arm, ArmModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstacleSDFFactorGPArm_ = shared_ptr[CObstacleSDFFactorGPArm](new CObstacleSDFFactorGPArm(pose1Key, vel1Key, pose2Key, vel2Key, deref(arm.CArmModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorGPArm_.use_count()==0):
            raise TypeError('ObstacleSDFFactorGPArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorGPArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorGPArm_)

    @staticmethod
    cdef ObstacleSDFFactorGPArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorGPArm return_value = ObstacleSDFFactorGPArm(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorGPArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstacleSDFFactorGPArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorGPArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPArm]>dynamic_pointer_cast[CObstacleSDFFactorGPArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorGPArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorGPArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPArm]>dynamic_pointer_cast[CObstacleSDFFactorGPArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorArm_ = shared_ptr[CObstaclePlanarSDFFactorArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'arm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            arm = <ArmModel>(__params[1])
            sdf = <PlanarSDF>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(arm, ArmModel)
            assert isinstance(sdf, PlanarSDF)
            self.CObstaclePlanarSDFFactorArm_ = shared_ptr[CObstaclePlanarSDFFactorArm](new CObstaclePlanarSDFFactorArm(posekey, deref(arm.CArmModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorArm_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorArm_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorArm return_value = ObstaclePlanarSDFFactorArm(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, np.ndarray pose):
        pose = pose.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CObstaclePlanarSDFFactorArm_.get().evaluateError(<VectorXd>(Map[VectorXd](pose)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstaclePlanarSDFFactorArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorGPArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorGPArm_ = shared_ptr[CObstaclePlanarSDFFactorGPArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'arm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            arm = <ArmModel>(__params[4])
            sdf = <PlanarSDF>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(arm, ArmModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstaclePlanarSDFFactorGPArm_ = shared_ptr[CObstaclePlanarSDFFactorGPArm](new CObstaclePlanarSDFFactorGPArm(pose1key, vel1key, pose2key, vel2key, deref(arm.CArmModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorGPArm_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorGPArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorGPArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorGPArm_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorGPArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorGPArm return_value = ObstaclePlanarSDFFactorGPArm(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorGPArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstaclePlanarSDFFactorGPArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorGPArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorPointRobot(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorPointRobot_ = shared_ptr[CObstaclePlanarSDFFactorPointRobot]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'pR', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            pR = <PointRobotModel>(__params[1])
            sdf = <PlanarSDF>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(pR, PointRobotModel)
            assert isinstance(sdf, PlanarSDF)
            self.CObstaclePlanarSDFFactorPointRobot_ = shared_ptr[CObstaclePlanarSDFFactorPointRobot](new CObstaclePlanarSDFFactorPointRobot(posekey, deref(pR.CPointRobotModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorPointRobot_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorPointRobot construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorPointRobot_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorPointRobot_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorPointRobot cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPointRobot]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorPointRobot return_value = ObstaclePlanarSDFFactorPointRobot(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorPointRobot_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, np.ndarray pose):
        pose = pose.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CObstaclePlanarSDFFactorPointRobot_.get().evaluateError(<VectorXd>(Map[VectorXd](pose)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstaclePlanarSDFFactorPointRobot_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorPointRobot.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPointRobot]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPointRobot,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorPointRobot_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorPointRobot.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPointRobot]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPointRobot,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorGPPointRobot(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorGPPointRobot_ = shared_ptr[CObstaclePlanarSDFFactorGPPointRobot]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'pR', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            pR = <PointRobotModel>(__params[4])
            sdf = <PlanarSDF>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(pR, PointRobotModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstaclePlanarSDFFactorGPPointRobot_ = shared_ptr[CObstaclePlanarSDFFactorGPPointRobot](new CObstaclePlanarSDFFactorGPPointRobot(pose1key, vel1key, pose2key, vel2key, deref(pR.CPointRobotModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorGPPointRobot_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorGPPointRobot construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorGPPointRobot_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorGPPointRobot_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPointRobot cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPointRobot]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorGPPointRobot return_value = ObstaclePlanarSDFFactorGPPointRobot(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorGPPointRobot_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstaclePlanarSDFFactorGPPointRobot_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPointRobot.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPointRobot]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPointRobot,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorGPPointRobot_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPointRobot.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPointRobot]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPointRobot,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorPose2MobileBase(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorPose2MobileBase_ = shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'robot', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            robot = <Pose2MobileBaseModel>(__params[1])
            sdf = <PlanarSDF>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(robot, Pose2MobileBaseModel)
            assert isinstance(sdf, PlanarSDF)
            self.CObstaclePlanarSDFFactorPose2MobileBase_ = shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase](new CObstaclePlanarSDFFactorPose2MobileBase(posekey, deref(robot.CPose2MobileBaseModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorPose2MobileBase_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorPose2MobileBase construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorPose2MobileBase_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorPose2MobileBase_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorPose2MobileBase cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorPose2MobileBase return_value = ObstaclePlanarSDFFactorPose2MobileBase(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorPose2MobileBase_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2 pose):
        cdef VectorXd ret = self.CObstaclePlanarSDFFactorPose2MobileBase_.get().evaluateError(deref(pose.CPose2_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstaclePlanarSDFFactorPose2MobileBase_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPose2MobileBase,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorPose2MobileBase_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPose2MobileBase]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPose2MobileBase,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorGPPose2MobileBase(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorGPPose2MobileBase_ = shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'robot', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            robot = <Pose2MobileBaseModel>(__params[4])
            sdf = <PlanarSDF>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(robot, Pose2MobileBaseModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstaclePlanarSDFFactorGPPose2MobileBase_ = shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase](new CObstaclePlanarSDFFactorGPPose2MobileBase(pose1key, vel1key, pose2key, vel2key, deref(robot.CPose2MobileBaseModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorGPPose2MobileBase_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorGPPose2MobileBase construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorGPPose2MobileBase_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorGPPose2MobileBase_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPose2MobileBase cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorGPPose2MobileBase return_value = ObstaclePlanarSDFFactorGPPose2MobileBase(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorGPPose2MobileBase_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstaclePlanarSDFFactorGPPose2MobileBase_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPose2MobileBase,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorGPPose2MobileBase_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileBase]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPose2MobileBase,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorPose2MobileArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorPose2MobileArm_ = shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'marm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            marm = <Pose2MobileArmModel>(__params[1])
            sdf = <PlanarSDF>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(marm, Pose2MobileArmModel)
            assert isinstance(sdf, PlanarSDF)
            self.CObstaclePlanarSDFFactorPose2MobileArm_ = shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm](new CObstaclePlanarSDFFactorPose2MobileArm(posekey, deref(marm.CPose2MobileArmModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorPose2MobileArm_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorPose2MobileArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorPose2MobileArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorPose2MobileArm_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorPose2MobileArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorPose2MobileArm return_value = ObstaclePlanarSDFFactorPose2MobileArm(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorPose2MobileArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2Vector pose):
        cdef VectorXd ret = self.CObstaclePlanarSDFFactorPose2MobileArm_.get().evaluateError(deref(pose.CPose2Vector_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstaclePlanarSDFFactorPose2MobileArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPose2MobileArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorPose2MobileArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPose2MobileArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPose2MobileArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorGPPose2MobileArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorGPPose2MobileArm_ = shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'marm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            marm = <Pose2MobileArmModel>(__params[4])
            sdf = <PlanarSDF>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(marm, Pose2MobileArmModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstaclePlanarSDFFactorGPPose2MobileArm_ = shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm](new CObstaclePlanarSDFFactorGPPose2MobileArm(pose1key, vel1key, pose2key, vel2key, deref(marm.CPose2MobileArmModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorGPPose2MobileArm_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorGPPose2MobileArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorGPPose2MobileArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorGPPose2MobileArm_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPose2MobileArm cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorGPPose2MobileArm return_value = ObstaclePlanarSDFFactorGPPose2MobileArm(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorGPPose2MobileArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstaclePlanarSDFFactorGPPose2MobileArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPose2MobileArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorGPPose2MobileArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPose2MobileArm]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPose2MobileArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorPose2Mobile2Arms(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorPose2Mobile2Arms_ = shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'marm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            marm = <Pose2Mobile2ArmsModel>(__params[1])
            sdf = <PlanarSDF>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(marm, Pose2Mobile2ArmsModel)
            assert isinstance(sdf, PlanarSDF)
            self.CObstaclePlanarSDFFactorPose2Mobile2Arms_ = shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms](new CObstaclePlanarSDFFactorPose2Mobile2Arms(posekey, deref(marm.CPose2Mobile2ArmsModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorPose2Mobile2Arms_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorPose2Mobile2Arms construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorPose2Mobile2Arms_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorPose2Mobile2Arms_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorPose2Mobile2Arms return_value = ObstaclePlanarSDFFactorPose2Mobile2Arms(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorPose2Mobile2Arms_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2Vector pose):
        cdef VectorXd ret = self.CObstaclePlanarSDFFactorPose2Mobile2Arms_.get().evaluateError(deref(pose.CPose2Vector_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstaclePlanarSDFFactorPose2Mobile2Arms_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPose2Mobile2Arms,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorPose2Mobile2Arms_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPose2Mobile2Arms]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPose2Mobile2Arms,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstaclePlanarSDFFactorGPPose2Mobile2Arms(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorGPPose2Mobile2Arms_ = shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'marm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            marm = <Pose2Mobile2ArmsModel>(__params[4])
            sdf = <PlanarSDF>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(marm, Pose2Mobile2ArmsModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstaclePlanarSDFFactorGPPose2Mobile2Arms_ = shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms](new CObstaclePlanarSDFFactorGPPose2Mobile2Arms(pose1key, vel1key, pose2key, vel2key, deref(marm.CPose2Mobile2ArmsModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorGPPose2Mobile2Arms_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorGPPose2Mobile2Arms construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorGPPose2Mobile2Arms_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorGPPose2Mobile2Arms_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorGPPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorGPPose2Mobile2Arms return_value = ObstaclePlanarSDFFactorGPPose2Mobile2Arms(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorGPPose2Mobile2Arms_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstaclePlanarSDFFactorGPPose2Mobile2Arms_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPose2Mobile2Arms,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorGPPose2Mobile2Arms_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorGPPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorGPPose2Mobile2Arms]>dynamic_pointer_cast[CObstaclePlanarSDFFactorGPPose2Mobile2Arms,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorPose2MobileBase(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorPose2MobileBase_ = shared_ptr[CObstacleSDFFactorPose2MobileBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'robot', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            robot = <Pose2MobileBaseModel>(__params[1])
            sdf = <SignedDistanceField>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(robot, Pose2MobileBaseModel)
            assert isinstance(sdf, SignedDistanceField)
            self.CObstacleSDFFactorPose2MobileBase_ = shared_ptr[CObstacleSDFFactorPose2MobileBase](new CObstacleSDFFactorPose2MobileBase(posekey, deref(robot.CPose2MobileBaseModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorPose2MobileBase_.use_count()==0):
            raise TypeError('ObstacleSDFFactorPose2MobileBase construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorPose2MobileBase_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorPose2MobileBase_)

    @staticmethod
    cdef ObstacleSDFFactorPose2MobileBase cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorPose2MobileBase return_value = ObstacleSDFFactorPose2MobileBase(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorPose2MobileBase_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2 pose):
        cdef VectorXd ret = self.CObstacleSDFFactorPose2MobileBase_.get().evaluateError(deref(pose.CPose2_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstacleSDFFactorPose2MobileBase_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileBase]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileBase,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorPose2MobileBase_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileBase]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileBase,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorGPPose2MobileBase(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorGPPose2MobileBase_ = shared_ptr[CObstacleSDFFactorGPPose2MobileBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'robot', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            robot = <Pose2MobileBaseModel>(__params[4])
            sdf = <SignedDistanceField>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(robot, Pose2MobileBaseModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstacleSDFFactorGPPose2MobileBase_ = shared_ptr[CObstacleSDFFactorGPPose2MobileBase](new CObstacleSDFFactorGPPose2MobileBase(pose1key, vel1key, pose2key, vel2key, deref(robot.CPose2MobileBaseModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorGPPose2MobileBase_.use_count()==0):
            raise TypeError('ObstacleSDFFactorGPPose2MobileBase construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorGPPose2MobileBase_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorGPPose2MobileBase_)

    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileBase cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorGPPose2MobileBase return_value = ObstacleSDFFactorGPPose2MobileBase(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorGPPose2MobileBase_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstacleSDFFactorGPPose2MobileBase_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileBase]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileBase,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorGPPose2MobileBase_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileBase.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileBase]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileBase,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorPose2MobileArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorPose2MobileArm_ = shared_ptr[CObstacleSDFFactorPose2MobileArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'marm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            marm = <Pose2MobileArmModel>(__params[1])
            sdf = <SignedDistanceField>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(marm, Pose2MobileArmModel)
            assert isinstance(sdf, SignedDistanceField)
            self.CObstacleSDFFactorPose2MobileArm_ = shared_ptr[CObstacleSDFFactorPose2MobileArm](new CObstacleSDFFactorPose2MobileArm(posekey, deref(marm.CPose2MobileArmModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorPose2MobileArm_.use_count()==0):
            raise TypeError('ObstacleSDFFactorPose2MobileArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorPose2MobileArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorPose2MobileArm_)

    @staticmethod
    cdef ObstacleSDFFactorPose2MobileArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorPose2MobileArm return_value = ObstacleSDFFactorPose2MobileArm(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorPose2MobileArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2Vector pose):
        cdef VectorXd ret = self.CObstacleSDFFactorPose2MobileArm_.get().evaluateError(deref(pose.CPose2Vector_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstacleSDFFactorPose2MobileArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileArm]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorPose2MobileArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileArm]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorGPPose2MobileArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorGPPose2MobileArm_ = shared_ptr[CObstacleSDFFactorGPPose2MobileArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'marm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            marm = <Pose2MobileArmModel>(__params[4])
            sdf = <SignedDistanceField>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(marm, Pose2MobileArmModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstacleSDFFactorGPPose2MobileArm_ = shared_ptr[CObstacleSDFFactorGPPose2MobileArm](new CObstacleSDFFactorGPPose2MobileArm(pose1key, vel1key, pose2key, vel2key, deref(marm.CPose2MobileArmModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorGPPose2MobileArm_.use_count()==0):
            raise TypeError('ObstacleSDFFactorGPPose2MobileArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorGPPose2MobileArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorGPPose2MobileArm_)

    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorGPPose2MobileArm return_value = ObstacleSDFFactorGPPose2MobileArm(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorGPPose2MobileArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstacleSDFFactorGPPose2MobileArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileArm]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorGPPose2MobileArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileArm]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorPose2Mobile2Arms(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorPose2Mobile2Arms_ = shared_ptr[CObstacleSDFFactorPose2Mobile2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'marm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            marm = <Pose2Mobile2ArmsModel>(__params[1])
            sdf = <SignedDistanceField>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(marm, Pose2Mobile2ArmsModel)
            assert isinstance(sdf, SignedDistanceField)
            self.CObstacleSDFFactorPose2Mobile2Arms_ = shared_ptr[CObstacleSDFFactorPose2Mobile2Arms](new CObstacleSDFFactorPose2Mobile2Arms(posekey, deref(marm.CPose2Mobile2ArmsModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorPose2Mobile2Arms_.use_count()==0):
            raise TypeError('ObstacleSDFFactorPose2Mobile2Arms construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorPose2Mobile2Arms_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorPose2Mobile2Arms_)

    @staticmethod
    cdef ObstacleSDFFactorPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2Mobile2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorPose2Mobile2Arms return_value = ObstacleSDFFactorPose2Mobile2Arms(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorPose2Mobile2Arms_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2Vector pose):
        cdef VectorXd ret = self.CObstacleSDFFactorPose2Mobile2Arms_.get().evaluateError(deref(pose.CPose2Vector_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstacleSDFFactorPose2Mobile2Arms_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2Mobile2Arms]>dynamic_pointer_cast[CObstacleSDFFactorPose2Mobile2Arms,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorPose2Mobile2Arms_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2Mobile2Arms]>dynamic_pointer_cast[CObstacleSDFFactorPose2Mobile2Arms,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorGPPose2Mobile2Arms(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorGPPose2Mobile2Arms_ = shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'marm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            marm = <Pose2Mobile2ArmsModel>(__params[4])
            sdf = <SignedDistanceField>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(marm, Pose2Mobile2ArmsModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstacleSDFFactorGPPose2Mobile2Arms_ = shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms](new CObstacleSDFFactorGPPose2Mobile2Arms(pose1key, vel1key, pose2key, vel2key, deref(marm.CPose2Mobile2ArmsModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorGPPose2Mobile2Arms_.use_count()==0):
            raise TypeError('ObstacleSDFFactorGPPose2Mobile2Arms construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorGPPose2Mobile2Arms_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorGPPose2Mobile2Arms_)

    @staticmethod
    cdef ObstacleSDFFactorGPPose2Mobile2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorGPPose2Mobile2Arms return_value = ObstacleSDFFactorGPPose2Mobile2Arms(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorGPPose2Mobile2Arms_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstacleSDFFactorGPPose2Mobile2Arms_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorGPPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2Mobile2Arms,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorGPPose2Mobile2Arms_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorGPPose2Mobile2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2Mobile2Arms]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2Mobile2Arms,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorPose2MobileVetLinArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorPose2MobileVetLinArm_ = shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'marm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            marm = <Pose2MobileVetLinArmModel>(__params[1])
            sdf = <SignedDistanceField>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(marm, Pose2MobileVetLinArmModel)
            assert isinstance(sdf, SignedDistanceField)
            self.CObstacleSDFFactorPose2MobileVetLinArm_ = shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm](new CObstacleSDFFactorPose2MobileVetLinArm(posekey, deref(marm.CPose2MobileVetLinArmModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorPose2MobileVetLinArm_.use_count()==0):
            raise TypeError('ObstacleSDFFactorPose2MobileVetLinArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorPose2MobileVetLinArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorPose2MobileVetLinArm_)

    @staticmethod
    cdef ObstacleSDFFactorPose2MobileVetLinArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorPose2MobileVetLinArm return_value = ObstacleSDFFactorPose2MobileVetLinArm(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorPose2MobileVetLinArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2Vector pose):
        cdef VectorXd ret = self.CObstacleSDFFactorPose2MobileVetLinArm_.get().evaluateError(deref(pose.CPose2Vector_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstacleSDFFactorPose2MobileVetLinArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileVetLinArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileVetLinArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorPose2MobileVetLinArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileVetLinArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileVetLinArm]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileVetLinArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorGPPose2MobileVetLinArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorGPPose2MobileVetLinArm_ = shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'marm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            marm = <Pose2MobileVetLinArmModel>(__params[4])
            sdf = <SignedDistanceField>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(marm, Pose2MobileVetLinArmModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstacleSDFFactorGPPose2MobileVetLinArm_ = shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm](new CObstacleSDFFactorGPPose2MobileVetLinArm(pose1key, vel1key, pose2key, vel2key, deref(marm.CPose2MobileVetLinArmModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorGPPose2MobileVetLinArm_.use_count()==0):
            raise TypeError('ObstacleSDFFactorGPPose2MobileVetLinArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorGPPose2MobileVetLinArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorGPPose2MobileVetLinArm_)

    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileVetLinArm cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorGPPose2MobileVetLinArm return_value = ObstacleSDFFactorGPPose2MobileVetLinArm(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorGPPose2MobileVetLinArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstacleSDFFactorGPPose2MobileVetLinArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileVetLinArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileVetLinArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorGPPose2MobileVetLinArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileVetLinArm.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileVetLinArm]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileVetLinArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorPose2MobileVetLin2Arms(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorPose2MobileVetLin2Arms_ = shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'marm', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            marm = <Pose2MobileVetLin2ArmsModel>(__params[1])
            sdf = <SignedDistanceField>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(marm, Pose2MobileVetLin2ArmsModel)
            assert isinstance(sdf, SignedDistanceField)
            self.CObstacleSDFFactorPose2MobileVetLin2Arms_ = shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms](new CObstacleSDFFactorPose2MobileVetLin2Arms(posekey, deref(marm.CPose2MobileVetLin2ArmsModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorPose2MobileVetLin2Arms_.use_count()==0):
            raise TypeError('ObstacleSDFFactorPose2MobileVetLin2Arms construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorPose2MobileVetLin2Arms_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorPose2MobileVetLin2Arms_)

    @staticmethod
    cdef ObstacleSDFFactorPose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorPose2MobileVetLin2Arms return_value = ObstacleSDFFactorPose2MobileVetLin2Arms(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorPose2MobileVetLin2Arms_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, Pose2Vector pose):
        cdef VectorXd ret = self.CObstacleSDFFactorPose2MobileVetLin2Arms_.get().evaluateError(deref(pose.CPose2Vector_))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstacleSDFFactorPose2MobileVetLin2Arms_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileVetLin2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileVetLin2Arms,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorPose2MobileVetLin2Arms_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorPose2MobileVetLin2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorPose2MobileVetLin2Arms]>dynamic_pointer_cast[CObstacleSDFFactorPose2MobileVetLin2Arms,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class ObstacleSDFFactorGPPose2MobileVetLin2Arms(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstacleSDFFactorGPPose2MobileVetLin2Arms_ = shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pose1key', 'vel1key', 'pose2key', 'vel2key', 'marm', 'sdf', 'cost_sigma', 'epsilon', 'Qc_model', 'delta_t', 'tau'], args, kwargs)
            pose1key = <size_t>(__params[0])
            vel1key = <size_t>(__params[1])
            pose2key = <size_t>(__params[2])
            vel2key = <size_t>(__params[3])
            marm = <Pose2MobileVetLin2ArmsModel>(__params[4])
            sdf = <SignedDistanceField>(__params[5])
            cost_sigma = <double>(__params[6])
            epsilon = <double>(__params[7])
            Qc_model = <noiseModel_Base>(__params[8])
            delta_t = <double>(__params[9])
            tau = <double>(__params[10])
            assert isinstance(marm, Pose2MobileVetLin2ArmsModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(Qc_model, noiseModel_Base)
            self.CObstacleSDFFactorGPPose2MobileVetLin2Arms_ = shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms](new CObstacleSDFFactorGPPose2MobileVetLin2Arms(pose1key, vel1key, pose2key, vel2key, deref(marm.CPose2MobileVetLin2ArmsModel_), deref(sdf.CSignedDistanceField_), cost_sigma, epsilon, Qc_model.CnoiseModel_Base_, delta_t, tau))
        except (AssertionError, ValueError):
            pass
        if (self.CObstacleSDFFactorGPPose2MobileVetLin2Arms_.use_count()==0):
            raise TypeError('ObstacleSDFFactorGPPose2MobileVetLin2Arms construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstacleSDFFactorGPPose2MobileVetLin2Arms_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstacleSDFFactorGPPose2MobileVetLin2Arms_)

    @staticmethod
    cdef ObstacleSDFFactorGPPose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstacleSDFFactorGPPose2MobileVetLin2Arms return_value = ObstacleSDFFactorGPPose2MobileVetLin2Arms(cyCreateFromShared=True)
        return_value.CObstacleSDFFactorGPPose2MobileVetLin2Arms_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_ObstacleSDFFactorGPPose2MobileVetLin2Arms_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileVetLin2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileVetLin2Arms,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstacleSDFFactorGPPose2MobileVetLin2Arms_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstacleSDFFactorGPPose2MobileVetLin2Arms.cyCreateFromShared(<shared_ptr[CObstacleSDFFactorGPPose2MobileVetLin2Arms]>dynamic_pointer_cast[CObstacleSDFFactorGPPose2MobileVetLin2Arms,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class SelfCollisionArm(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CSelfCollisionArm_ = shared_ptr[CSelfCollisionArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'arm', 'data'], args, kwargs)
            poseKey = <size_t>(__params[0])
            arm = <ArmModel>(__params[1])
            data = <np.ndarray>(__params[2])
            assert isinstance(arm, ArmModel)
            assert isinstance(data, np.ndarray) and data.ndim == 2
            data = data.astype(float, order='F', copy=False)
            self.CSelfCollisionArm_ = shared_ptr[CSelfCollisionArm](new CSelfCollisionArm(poseKey, deref(arm.CArmModel_), <MatrixXd>(Map[MatrixXd](data))))
        except (AssertionError, ValueError):
            pass
        if (self.CSelfCollisionArm_.use_count()==0):
            raise TypeError('SelfCollisionArm construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CSelfCollisionArm_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CSelfCollisionArm_)

    @staticmethod
    cdef SelfCollisionArm cyCreateFromShared(const shared_ptr[CSelfCollisionArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef SelfCollisionArm return_value = SelfCollisionArm(cyCreateFromShared=True)
        return_value.CSelfCollisionArm_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, np.ndarray pose):
        pose = pose.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CSelfCollisionArm_.get().evaluateError(<VectorXd>(Map[VectorXd](pose)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_SelfCollisionArm_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return SelfCollisionArm.cyCreateFromShared(<shared_ptr[CSelfCollisionArm]>dynamic_pointer_cast[CSelfCollisionArm,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_SelfCollisionArm_NonlinearFactor(NonlinearFactor parent):
    try:
        return SelfCollisionArm.cyCreateFromShared(<shared_ptr[CSelfCollisionArm]>dynamic_pointer_cast[CSelfCollisionArm,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class TrajOptimizerSetting:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CTrajOptimizerSetting_ = shared_ptr[CTrajOptimizerSetting]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['dof'], args, kwargs)
            dof = <size_t>(__params[0])
            self.CTrajOptimizerSetting_ = shared_ptr[CTrajOptimizerSetting](new CTrajOptimizerSetting(dof))
        except (AssertionError, ValueError):
            pass
        if (self.CTrajOptimizerSetting_.use_count()==0):
            raise TypeError('TrajOptimizerSetting construction failed!')

    @staticmethod
    cdef TrajOptimizerSetting cyCreateFromShared(const shared_ptr[CTrajOptimizerSetting]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef TrajOptimizerSetting return_value = TrajOptimizerSetting(cyCreateFromShared=True)
        return_value.CTrajOptimizerSetting_ = other
        return return_value

    def setDogleg(self):
        self.CTrajOptimizerSetting_.get().setDogleg()
    def setGaussNewton(self):
        self.CTrajOptimizerSetting_.get().setGaussNewton()
    def setLM(self):
        self.CTrajOptimizerSetting_.get().setLM()
    def setOptimizationNoIncrase(self, bool flag):
        self.CTrajOptimizerSetting_.get().setOptimizationNoIncrase(flag)
    def setVerbosityError(self):
        self.CTrajOptimizerSetting_.get().setVerbosityError()
    def setVerbosityNone(self):
        self.CTrajOptimizerSetting_.get().setVerbosityNone()
    def set_Qc_model(self, np.ndarray Qc):
        Qc = Qc.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_Qc_model(<MatrixXd>(Map[MatrixXd](Qc)))
    def set_conf_prior_model(self, double sigma):
        self.CTrajOptimizerSetting_.get().set_conf_prior_model(sigma)
    def set_cost_sigma(self, double sigma):
        self.CTrajOptimizerSetting_.get().set_cost_sigma(sigma)
    def set_epsilon(self, double eps):
        self.CTrajOptimizerSetting_.get().set_epsilon(eps)
    def set_flag_pos_limit(self, bool flag):
        self.CTrajOptimizerSetting_.get().set_flag_pos_limit(flag)
    def set_flag_vel_limit(self, bool flag):
        self.CTrajOptimizerSetting_.get().set_flag_vel_limit(flag)
    def set_joint_pos_limits_down(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_joint_pos_limits_down(<VectorXd>(Map[VectorXd](v)))
    def set_joint_pos_limits_up(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_joint_pos_limits_up(<VectorXd>(Map[VectorXd](v)))
    def set_max_iter(self, size_t iter):
        self.CTrajOptimizerSetting_.get().set_max_iter(iter)
    def set_obs_check_inter(self, size_t inter):
        self.CTrajOptimizerSetting_.get().set_obs_check_inter(inter)
    def set_pos_limit_model(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_pos_limit_model(<VectorXd>(Map[VectorXd](v)))
    def set_pos_limit_thresh(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_pos_limit_thresh(<VectorXd>(Map[VectorXd](v)))
    def set_rel_thresh(self, double thresh):
        self.CTrajOptimizerSetting_.get().set_rel_thresh(thresh)
    def set_total_step(self, size_t step):
        self.CTrajOptimizerSetting_.get().set_total_step(step)
    def set_total_time(self, double time):
        self.CTrajOptimizerSetting_.get().set_total_time(time)
    def set_vel_limit_model(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_vel_limit_model(<VectorXd>(Map[VectorXd](v)))
    def set_vel_limit_thresh(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_vel_limit_thresh(<VectorXd>(Map[VectorXd](v)))
    def set_vel_limits(self, np.ndarray v):
        v = v.astype(float, order='F', copy=False)
        self.CTrajOptimizerSetting_.get().set_vel_limits(<VectorXd>(Map[VectorXd](v)))
    def set_vel_prior_model(self, double sigma):
        self.CTrajOptimizerSetting_.get().set_vel_prior_model(sigma)


cdef class ISAM2TrajOptimizer2DArm:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CISAM2TrajOptimizer2DArm_ = shared_ptr[CISAM2TrajOptimizer2DArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm', 'sdf', 'setting'], args, kwargs)
            arm = <ArmModel>(__params[0])
            sdf = <PlanarSDF>(__params[1])
            setting = <TrajOptimizerSetting>(__params[2])
            assert isinstance(arm, ArmModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(setting, TrajOptimizerSetting)
            self.CISAM2TrajOptimizer2DArm_ = shared_ptr[CISAM2TrajOptimizer2DArm](new CISAM2TrajOptimizer2DArm(deref(arm.CArmModel_), deref(sdf.CPlanarSDF_), deref(setting.CTrajOptimizerSetting_)))
        except (AssertionError, ValueError):
            pass
        if (self.CISAM2TrajOptimizer2DArm_.use_count()==0):
            raise TypeError('ISAM2TrajOptimizer2DArm construction failed!')

    @staticmethod
    cdef ISAM2TrajOptimizer2DArm cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizer2DArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ISAM2TrajOptimizer2DArm return_value = ISAM2TrajOptimizer2DArm(cyCreateFromShared=True)
        return_value.CISAM2TrajOptimizer2DArm_ = other
        return return_value

    def addPoseEstimate(self, size_t state_idx, np.ndarray pose, np.ndarray pose_cov):
        pose = pose.astype(float, order='F', copy=False)
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer2DArm_.get().addPoseEstimate(state_idx, <VectorXd>(Map[VectorXd](pose)), <MatrixXd>(Map[MatrixXd](pose_cov)))
    def addStateEstimate(self, size_t state_idx, np.ndarray pose, np.ndarray pose_cov, np.ndarray vel, np.ndarray vel_cov):
        pose = pose.astype(float, order='F', copy=False)
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        vel = vel.astype(float, order='F', copy=False)
        vel_cov = vel_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer2DArm_.get().addStateEstimate(state_idx, <VectorXd>(Map[VectorXd](pose)), <MatrixXd>(Map[MatrixXd](pose_cov)), <VectorXd>(Map[VectorXd](vel)), <MatrixXd>(Map[MatrixXd](vel_cov)))
    def changeGoalConfigAndVel(self, np.ndarray goal_conf, np.ndarray goal_vel):
        goal_conf = goal_conf.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer2DArm_.get().changeGoalConfigAndVel(<VectorXd>(Map[VectorXd](goal_conf)), <VectorXd>(Map[VectorXd](goal_vel)))
    def fixConfigAndVel(self, size_t state_idx, np.ndarray conf_fix, np.ndarray vel_fix):
        conf_fix = conf_fix.astype(float, order='F', copy=False)
        vel_fix = vel_fix.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer2DArm_.get().fixConfigAndVel(state_idx, <VectorXd>(Map[VectorXd](conf_fix)), <VectorXd>(Map[VectorXd](vel_fix)))
    def initFactorGraph(self, np.ndarray start_conf, np.ndarray start_vel, np.ndarray goal_conf, np.ndarray goal_vel):
        start_conf = start_conf.astype(float, order='F', copy=False)
        start_vel = start_vel.astype(float, order='F', copy=False)
        goal_conf = goal_conf.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer2DArm_.get().initFactorGraph(<VectorXd>(Map[VectorXd](start_conf)), <VectorXd>(Map[VectorXd](start_vel)), <VectorXd>(Map[VectorXd](goal_conf)), <VectorXd>(Map[VectorXd](goal_vel)))
    def initValues(self, Values init_values):
        self.CISAM2TrajOptimizer2DArm_.get().initValues(deref(init_values.CValues_))
    def removeGoalConfigAndVel(self):
        self.CISAM2TrajOptimizer2DArm_.get().removeGoalConfigAndVel()
    def update(self):
        self.CISAM2TrajOptimizer2DArm_.get().update()
    def values(self):
        cdef shared_ptr[CValues] ret = make_shared[CValues](self.CISAM2TrajOptimizer2DArm_.get().values())
        return Values.cyCreateFromShared(ret)


cdef class ISAM2TrajOptimizer3DArm:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CISAM2TrajOptimizer3DArm_ = shared_ptr[CISAM2TrajOptimizer3DArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['arm', 'sdf', 'setting'], args, kwargs)
            arm = <ArmModel>(__params[0])
            sdf = <SignedDistanceField>(__params[1])
            setting = <TrajOptimizerSetting>(__params[2])
            assert isinstance(arm, ArmModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(setting, TrajOptimizerSetting)
            self.CISAM2TrajOptimizer3DArm_ = shared_ptr[CISAM2TrajOptimizer3DArm](new CISAM2TrajOptimizer3DArm(deref(arm.CArmModel_), deref(sdf.CSignedDistanceField_), deref(setting.CTrajOptimizerSetting_)))
        except (AssertionError, ValueError):
            pass
        if (self.CISAM2TrajOptimizer3DArm_.use_count()==0):
            raise TypeError('ISAM2TrajOptimizer3DArm construction failed!')

    @staticmethod
    cdef ISAM2TrajOptimizer3DArm cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizer3DArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ISAM2TrajOptimizer3DArm return_value = ISAM2TrajOptimizer3DArm(cyCreateFromShared=True)
        return_value.CISAM2TrajOptimizer3DArm_ = other
        return return_value

    def addPoseEstimate(self, size_t state_idx, np.ndarray pose, np.ndarray pose_cov):
        pose = pose.astype(float, order='F', copy=False)
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer3DArm_.get().addPoseEstimate(state_idx, <VectorXd>(Map[VectorXd](pose)), <MatrixXd>(Map[MatrixXd](pose_cov)))
    def addStateEstimate(self, size_t state_idx, np.ndarray pose, np.ndarray pose_cov, np.ndarray vel, np.ndarray vel_cov):
        pose = pose.astype(float, order='F', copy=False)
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        vel = vel.astype(float, order='F', copy=False)
        vel_cov = vel_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer3DArm_.get().addStateEstimate(state_idx, <VectorXd>(Map[VectorXd](pose)), <MatrixXd>(Map[MatrixXd](pose_cov)), <VectorXd>(Map[VectorXd](vel)), <MatrixXd>(Map[MatrixXd](vel_cov)))
    def changeGoalConfigAndVel(self, np.ndarray goal_conf, np.ndarray goal_vel):
        goal_conf = goal_conf.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer3DArm_.get().changeGoalConfigAndVel(<VectorXd>(Map[VectorXd](goal_conf)), <VectorXd>(Map[VectorXd](goal_vel)))
    def fixConfigAndVel(self, size_t state_idx, np.ndarray conf_fix, np.ndarray vel_fix):
        conf_fix = conf_fix.astype(float, order='F', copy=False)
        vel_fix = vel_fix.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer3DArm_.get().fixConfigAndVel(state_idx, <VectorXd>(Map[VectorXd](conf_fix)), <VectorXd>(Map[VectorXd](vel_fix)))
    def initFactorGraph(self, np.ndarray start_conf, np.ndarray start_vel, np.ndarray goal_conf, np.ndarray goal_vel):
        start_conf = start_conf.astype(float, order='F', copy=False)
        start_vel = start_vel.astype(float, order='F', copy=False)
        goal_conf = goal_conf.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizer3DArm_.get().initFactorGraph(<VectorXd>(Map[VectorXd](start_conf)), <VectorXd>(Map[VectorXd](start_vel)), <VectorXd>(Map[VectorXd](goal_conf)), <VectorXd>(Map[VectorXd](goal_vel)))
    def initValues(self, Values init_values):
        self.CISAM2TrajOptimizer3DArm_.get().initValues(deref(init_values.CValues_))
    def removeGoalConfigAndVel(self):
        self.CISAM2TrajOptimizer3DArm_.get().removeGoalConfigAndVel()
    def update(self):
        self.CISAM2TrajOptimizer3DArm_.get().update()
    def values(self):
        cdef shared_ptr[CValues] ret = make_shared[CValues](self.CISAM2TrajOptimizer3DArm_.get().values())
        return Values.cyCreateFromShared(ret)


cdef class ISAM2TrajOptimizerPose2MobileArm2D:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CISAM2TrajOptimizerPose2MobileArm2D_ = shared_ptr[CISAM2TrajOptimizerPose2MobileArm2D]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['marm', 'sdf', 'setting'], args, kwargs)
            marm = <Pose2MobileArmModel>(__params[0])
            sdf = <PlanarSDF>(__params[1])
            setting = <TrajOptimizerSetting>(__params[2])
            assert isinstance(marm, Pose2MobileArmModel)
            assert isinstance(sdf, PlanarSDF)
            assert isinstance(setting, TrajOptimizerSetting)
            self.CISAM2TrajOptimizerPose2MobileArm2D_ = shared_ptr[CISAM2TrajOptimizerPose2MobileArm2D](new CISAM2TrajOptimizerPose2MobileArm2D(deref(marm.CPose2MobileArmModel_), deref(sdf.CPlanarSDF_), deref(setting.CTrajOptimizerSetting_)))
        except (AssertionError, ValueError):
            pass
        if (self.CISAM2TrajOptimizerPose2MobileArm2D_.use_count()==0):
            raise TypeError('ISAM2TrajOptimizerPose2MobileArm2D construction failed!')

    @staticmethod
    cdef ISAM2TrajOptimizerPose2MobileArm2D cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizerPose2MobileArm2D]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ISAM2TrajOptimizerPose2MobileArm2D return_value = ISAM2TrajOptimizerPose2MobileArm2D(cyCreateFromShared=True)
        return_value.CISAM2TrajOptimizerPose2MobileArm2D_ = other
        return return_value

    def addPoseEstimate(self, size_t state_idx, Pose2Vector pose, np.ndarray pose_cov):
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().addPoseEstimate(state_idx, deref(pose.CPose2Vector_), <MatrixXd>(Map[MatrixXd](pose_cov)))
    def addStateEstimate(self, size_t state_idx, Pose2Vector pose, np.ndarray pose_cov, np.ndarray vel, np.ndarray vel_cov):
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        vel = vel.astype(float, order='F', copy=False)
        vel_cov = vel_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().addStateEstimate(state_idx, deref(pose.CPose2Vector_), <MatrixXd>(Map[MatrixXd](pose_cov)), <VectorXd>(Map[VectorXd](vel)), <MatrixXd>(Map[MatrixXd](vel_cov)))
    def changeGoalConfigAndVel(self, Pose2Vector goal_conf, np.ndarray goal_vel):
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().changeGoalConfigAndVel(deref(goal_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](goal_vel)))
    def fixConfigAndVel(self, size_t state_idx, Pose2Vector conf_fix, np.ndarray vel_fix):
        vel_fix = vel_fix.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().fixConfigAndVel(state_idx, deref(conf_fix.CPose2Vector_), <VectorXd>(Map[VectorXd](vel_fix)))
    def initFactorGraph(self, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector goal_conf, np.ndarray goal_vel):
        start_vel = start_vel.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().initFactorGraph(deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(goal_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](goal_vel)))
    def initValues(self, Values init_values):
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().initValues(deref(init_values.CValues_))
    def removeGoalConfigAndVel(self):
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().removeGoalConfigAndVel()
    def update(self):
        self.CISAM2TrajOptimizerPose2MobileArm2D_.get().update()
    def values(self):
        cdef shared_ptr[CValues] ret = make_shared[CValues](self.CISAM2TrajOptimizerPose2MobileArm2D_.get().values())
        return Values.cyCreateFromShared(ret)


cdef class ISAM2TrajOptimizerPose2MobileArm:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CISAM2TrajOptimizerPose2MobileArm_ = shared_ptr[CISAM2TrajOptimizerPose2MobileArm]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['marm', 'sdf', 'setting'], args, kwargs)
            marm = <Pose2MobileArmModel>(__params[0])
            sdf = <SignedDistanceField>(__params[1])
            setting = <TrajOptimizerSetting>(__params[2])
            assert isinstance(marm, Pose2MobileArmModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(setting, TrajOptimizerSetting)
            self.CISAM2TrajOptimizerPose2MobileArm_ = shared_ptr[CISAM2TrajOptimizerPose2MobileArm](new CISAM2TrajOptimizerPose2MobileArm(deref(marm.CPose2MobileArmModel_), deref(sdf.CSignedDistanceField_), deref(setting.CTrajOptimizerSetting_)))
        except (AssertionError, ValueError):
            pass
        if (self.CISAM2TrajOptimizerPose2MobileArm_.use_count()==0):
            raise TypeError('ISAM2TrajOptimizerPose2MobileArm construction failed!')

    @staticmethod
    cdef ISAM2TrajOptimizerPose2MobileArm cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizerPose2MobileArm]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ISAM2TrajOptimizerPose2MobileArm return_value = ISAM2TrajOptimizerPose2MobileArm(cyCreateFromShared=True)
        return_value.CISAM2TrajOptimizerPose2MobileArm_ = other
        return return_value

    def addPoseEstimate(self, size_t state_idx, Pose2Vector pose, np.ndarray pose_cov):
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm_.get().addPoseEstimate(state_idx, deref(pose.CPose2Vector_), <MatrixXd>(Map[MatrixXd](pose_cov)))
    def addStateEstimate(self, size_t state_idx, Pose2Vector pose, np.ndarray pose_cov, np.ndarray vel, np.ndarray vel_cov):
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        vel = vel.astype(float, order='F', copy=False)
        vel_cov = vel_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm_.get().addStateEstimate(state_idx, deref(pose.CPose2Vector_), <MatrixXd>(Map[MatrixXd](pose_cov)), <VectorXd>(Map[VectorXd](vel)), <MatrixXd>(Map[MatrixXd](vel_cov)))
    def changeGoalConfigAndVel(self, Pose2Vector goal_conf, np.ndarray goal_vel):
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm_.get().changeGoalConfigAndVel(deref(goal_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](goal_vel)))
    def fixConfigAndVel(self, size_t state_idx, Pose2Vector conf_fix, np.ndarray vel_fix):
        vel_fix = vel_fix.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm_.get().fixConfigAndVel(state_idx, deref(conf_fix.CPose2Vector_), <VectorXd>(Map[VectorXd](vel_fix)))
    def initFactorGraph(self, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector goal_conf, np.ndarray goal_vel):
        start_vel = start_vel.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileArm_.get().initFactorGraph(deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(goal_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](goal_vel)))
    def initValues(self, Values init_values):
        self.CISAM2TrajOptimizerPose2MobileArm_.get().initValues(deref(init_values.CValues_))
    def removeGoalConfigAndVel(self):
        self.CISAM2TrajOptimizerPose2MobileArm_.get().removeGoalConfigAndVel()
    def update(self):
        self.CISAM2TrajOptimizerPose2MobileArm_.get().update()
    def values(self):
        cdef shared_ptr[CValues] ret = make_shared[CValues](self.CISAM2TrajOptimizerPose2MobileArm_.get().values())
        return Values.cyCreateFromShared(ret)


cdef class ISAM2TrajOptimizerPose2MobileVetLin2Arms:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_ = shared_ptr[CISAM2TrajOptimizerPose2MobileVetLin2Arms]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['marm', 'sdf', 'setting'], args, kwargs)
            marm = <Pose2MobileVetLin2ArmsModel>(__params[0])
            sdf = <SignedDistanceField>(__params[1])
            setting = <TrajOptimizerSetting>(__params[2])
            assert isinstance(marm, Pose2MobileVetLin2ArmsModel)
            assert isinstance(sdf, SignedDistanceField)
            assert isinstance(setting, TrajOptimizerSetting)
            self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_ = shared_ptr[CISAM2TrajOptimizerPose2MobileVetLin2Arms](new CISAM2TrajOptimizerPose2MobileVetLin2Arms(deref(marm.CPose2MobileVetLin2ArmsModel_), deref(sdf.CSignedDistanceField_), deref(setting.CTrajOptimizerSetting_)))
        except (AssertionError, ValueError):
            pass
        if (self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.use_count()==0):
            raise TypeError('ISAM2TrajOptimizerPose2MobileVetLin2Arms construction failed!')

    @staticmethod
    cdef ISAM2TrajOptimizerPose2MobileVetLin2Arms cyCreateFromShared(const shared_ptr[CISAM2TrajOptimizerPose2MobileVetLin2Arms]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ISAM2TrajOptimizerPose2MobileVetLin2Arms return_value = ISAM2TrajOptimizerPose2MobileVetLin2Arms(cyCreateFromShared=True)
        return_value.CISAM2TrajOptimizerPose2MobileVetLin2Arms_ = other
        return return_value

    def addPoseEstimate(self, size_t state_idx, Pose2Vector pose, np.ndarray pose_cov):
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().addPoseEstimate(state_idx, deref(pose.CPose2Vector_), <MatrixXd>(Map[MatrixXd](pose_cov)))
    def addStateEstimate(self, size_t state_idx, Pose2Vector pose, np.ndarray pose_cov, np.ndarray vel, np.ndarray vel_cov):
        pose_cov = pose_cov.astype(float, order='F', copy=False)
        vel = vel.astype(float, order='F', copy=False)
        vel_cov = vel_cov.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().addStateEstimate(state_idx, deref(pose.CPose2Vector_), <MatrixXd>(Map[MatrixXd](pose_cov)), <VectorXd>(Map[VectorXd](vel)), <MatrixXd>(Map[MatrixXd](vel_cov)))
    def changeGoalConfigAndVel(self, Pose2Vector goal_conf, np.ndarray goal_vel):
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().changeGoalConfigAndVel(deref(goal_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](goal_vel)))
    def fixConfigAndVel(self, size_t state_idx, Pose2Vector conf_fix, np.ndarray vel_fix):
        vel_fix = vel_fix.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().fixConfigAndVel(state_idx, deref(conf_fix.CPose2Vector_), <VectorXd>(Map[VectorXd](vel_fix)))
    def initFactorGraph(self, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector goal_conf, np.ndarray goal_vel):
        start_vel = start_vel.astype(float, order='F', copy=False)
        goal_vel = goal_vel.astype(float, order='F', copy=False)
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().initFactorGraph(deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(goal_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](goal_vel)))
    def initValues(self, Values init_values):
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().initValues(deref(init_values.CValues_))
    def removeGoalConfigAndVel(self):
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().removeGoalConfigAndVel()
    def update(self):
        self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().update()
    def values(self):
        cdef shared_ptr[CValues] ret = make_shared[CValues](self.CISAM2TrajOptimizerPose2MobileVetLin2Arms_.get().values())
        return Values.cyCreateFromShared(ret)


cdef class PriorFactorPose2Vector(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPriorFactorPose2Vector_ = shared_ptr[CPriorFactorPose2Vector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['poseKey', 'value', 'model'], args, kwargs)
            poseKey = <size_t>(__params[0])
            value = <Pose2Vector>(__params[1])
            model = <noiseModel_Base>(__params[2])
            assert isinstance(value, Pose2Vector)
            assert isinstance(model, noiseModel_Base)
            self.CPriorFactorPose2Vector_ = shared_ptr[CPriorFactorPose2Vector](new CPriorFactorPose2Vector(poseKey, deref(value.CPose2Vector_), model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CPriorFactorPose2Vector_.use_count()==0):
            raise TypeError('PriorFactorPose2Vector construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CPriorFactorPose2Vector_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CPriorFactorPose2Vector_)

    @staticmethod
    cdef PriorFactorPose2Vector cyCreateFromShared(const shared_ptr[CPriorFactorPose2Vector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PriorFactorPose2Vector return_value = PriorFactorPose2Vector(cyCreateFromShared=True)
        return_value.CPriorFactorPose2Vector_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

def dynamic_cast_PriorFactorPose2Vector_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return PriorFactorPose2Vector.cyCreateFromShared(<shared_ptr[CPriorFactorPose2Vector]>dynamic_pointer_cast[CPriorFactorPose2Vector,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_PriorFactorPose2Vector_NonlinearFactor(NonlinearFactor parent):
    try:
        return PriorFactorPose2Vector.cyCreateFromShared(<shared_ptr[CPriorFactorPose2Vector]>dynamic_pointer_cast[CPriorFactorPose2Vector,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')



def BatchTrajOptimize2DArm(ArmModel arm, PlanarSDF sdf, np.ndarray start_conf, np.ndarray start_vel, np.ndarray end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_conf = start_conf.astype(float, order='F', copy=False)
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_conf = end_conf.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimize2DArm(deref(arm.CArmModel_), deref(sdf.CPlanarSDF_), <VectorXd>(Map[VectorXd](start_conf)), <VectorXd>(Map[VectorXd](start_vel)), <VectorXd>(Map[VectorXd](end_conf)), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def BatchTrajOptimize3DArm(ArmModel arm, SignedDistanceField sdf, np.ndarray start_conf, np.ndarray start_vel, np.ndarray end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_conf = start_conf.astype(float, order='F', copy=False)
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_conf = end_conf.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimize3DArm(deref(arm.CArmModel_), deref(sdf.CSignedDistanceField_), <VectorXd>(Map[VectorXd](start_conf)), <VectorXd>(Map[VectorXd](start_vel)), <VectorXd>(Map[VectorXd](end_conf)), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def BatchTrajOptimizePose2Mobile2Arms(Pose2Mobile2ArmsModel marm, SignedDistanceField sdf, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimizePose2Mobile2Arms(deref(marm.CPose2Mobile2ArmsModel_), deref(sdf.CSignedDistanceField_), deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(end_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def BatchTrajOptimizePose2MobileArm(Pose2MobileArmModel marm, SignedDistanceField sdf, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimizePose2MobileArm(deref(marm.CPose2MobileArmModel_), deref(sdf.CSignedDistanceField_), deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(end_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def BatchTrajOptimizePose2MobileArm2D(Pose2MobileArmModel marm, PlanarSDF sdf, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimizePose2MobileArm2D(deref(marm.CPose2MobileArmModel_), deref(sdf.CPlanarSDF_), deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(end_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def BatchTrajOptimizePose2MobileVetLin2Arms(Pose2MobileVetLin2ArmsModel marm, SignedDistanceField sdf, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimizePose2MobileVetLin2Arms(deref(marm.CPose2MobileVetLin2ArmsModel_), deref(sdf.CSignedDistanceField_), deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(end_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def BatchTrajOptimizePose2MobileVetLinArm(Pose2MobileVetLinArmModel marm, SignedDistanceField sdf, Pose2Vector start_conf, np.ndarray start_vel, Pose2Vector end_conf, np.ndarray end_vel, Values init_values, TrajOptimizerSetting setting):
    start_vel = start_vel.astype(float, order='F', copy=False)
    end_vel = end_vel.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_BatchTrajOptimizePose2MobileVetLinArm(deref(marm.CPose2MobileVetLinArmModel_), deref(sdf.CSignedDistanceField_), deref(start_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](start_vel)), deref(end_conf.CPose2Vector_), <VectorXd>(Map[VectorXd](end_vel)), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_)))
    return Values.cyCreateFromShared(ret)
def CollisionCost2DArm(ArmModel arm, PlanarSDF sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCost2DArm(deref(arm.CArmModel_), deref(sdf.CPlanarSDF_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCost3DArm(ArmModel arm, SignedDistanceField sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCost3DArm(deref(arm.CArmModel_), deref(sdf.CSignedDistanceField_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2Mobile2Arms(Pose2Mobile2ArmsModel marm, SignedDistanceField sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2Mobile2Arms(deref(marm.CPose2Mobile2ArmsModel_), deref(sdf.CSignedDistanceField_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2MobileArm(Pose2MobileArmModel marm, SignedDistanceField sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2MobileArm(deref(marm.CPose2MobileArmModel_), deref(sdf.CSignedDistanceField_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2MobileArm2D(Pose2MobileArmModel marm, PlanarSDF sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2MobileArm2D(deref(marm.CPose2MobileArmModel_), deref(sdf.CPlanarSDF_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2MobileBase(Pose2MobileBaseModel robot, SignedDistanceField sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2MobileBase(deref(robot.CPose2MobileBaseModel_), deref(sdf.CSignedDistanceField_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2MobileBase2D(Pose2MobileBaseModel robot, PlanarSDF sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2MobileBase2D(deref(robot.CPose2MobileBaseModel_), deref(sdf.CPlanarSDF_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2MobileVetLin2Arms(Pose2MobileVetLin2ArmsModel marm, SignedDistanceField sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2MobileVetLin2Arms(deref(marm.CPose2MobileVetLin2ArmsModel_), deref(sdf.CSignedDistanceField_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def CollisionCostPose2MobileVetLinArm(Pose2MobileVetLinArmModel marm, SignedDistanceField sdf, Values result, TrajOptimizerSetting setting):
    cdef double ret = pxd_CollisionCostPose2MobileVetLinArm(deref(marm.CPose2MobileVetLinArmModel_), deref(sdf.CSignedDistanceField_), deref(result.CValues_), deref(setting.CTrajOptimizerSetting_))
    return ret
def atPose2VectorValues(size_t key, Values values):
    cdef shared_ptr[CPose2Vector] ret = make_shared[CPose2Vector](pxd_atPose2VectorValues(key, deref(values.CValues_)))
    return Pose2Vector.cyCreateFromShared(ret)
def initArmTrajStraightLine(np.ndarray init_conf, np.ndarray end_conf, size_t total_step):
    init_conf = init_conf.astype(float, order='F', copy=False)
    end_conf = end_conf.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_initArmTrajStraightLine(<VectorXd>(Map[VectorXd](init_conf)), <VectorXd>(Map[VectorXd](end_conf)), total_step))
    return Values.cyCreateFromShared(ret)
def initPose2TrajStraightLine(Pose2 init_pose, Pose2 end_pose, size_t total_step):
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_initPose2TrajStraightLine(deref(init_pose.CPose2_), deref(end_pose.CPose2_), total_step))
    return Values.cyCreateFromShared(ret)
def initPose2VectorTrajStraightLine(Pose2 init_pose, np.ndarray init_conf, Pose2 end_pose, np.ndarray end_conf, size_t total_step):
    init_conf = init_conf.astype(float, order='F', copy=False)
    end_conf = end_conf.astype(float, order='F', copy=False)
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_initPose2VectorTrajStraightLine(deref(init_pose.CPose2_), <VectorXd>(Map[VectorXd](init_conf)), deref(end_pose.CPose2_), <VectorXd>(Map[VectorXd](end_conf)), total_step))
    return Values.cyCreateFromShared(ret)
def insertPose2VectorInValues(size_t key, Pose2Vector p, Values values):
    pxd_insertPose2VectorInValues(key, deref(p.CPose2Vector_), deref(values.CValues_))
def interpolateArmTraj(*args, **kwargs):
    success, results = interpolateArmTraj_0(args, kwargs)
    if success:
            return results
    success, results = interpolateArmTraj_1(args, kwargs)
    if success:
            return results
    raise TypeError('Could not find the correct overload')
def interpolateArmTraj_0(args, kwargs):
    cdef list __params
    cdef shared_ptr[CValues] return_value
    try:
        __params = process_args(['opt_values', 'Qc_model', 'delta_t', 'inter_step'], args, kwargs)
        opt_values = <Values>(__params[0])
        Qc_model = <noiseModel_Base>(__params[1])
        delta_t = <double>(__params[2])
        inter_step = <size_t>(__params[3])
        assert isinstance(opt_values, Values)
        assert isinstance(Qc_model, noiseModel_Base)
        return_value = make_shared[CValues](pxd_interpolateArmTraj(deref(opt_values.CValues_), Qc_model.CnoiseModel_Base_, delta_t, inter_step))
        return True, Values.cyCreateFromShared(return_value)
    except:
        return False, None

def interpolateArmTraj_1(args, kwargs):
    cdef list __params
    cdef shared_ptr[CValues] return_value
    try:
        __params = process_args(['opt_values', 'Qc_model', 'delta_t', 'inter_step', 'start_index', 'end_index'], args, kwargs)
        opt_values = <Values>(__params[0])
        Qc_model = <noiseModel_Base>(__params[1])
        delta_t = <double>(__params[2])
        inter_step = <size_t>(__params[3])
        start_index = <size_t>(__params[4])
        end_index = <size_t>(__params[5])
        assert isinstance(opt_values, Values)
        assert isinstance(Qc_model, noiseModel_Base)
        return_value = make_shared[CValues](pxd_interpolateArmTraj(deref(opt_values.CValues_), Qc_model.CnoiseModel_Base_, delta_t, inter_step, start_index, end_index))
        return True, Values.cyCreateFromShared(return_value)
    except:
        return False, None

def interpolatePose2MobileArmTraj(Values opt_values, noiseModel_Base Qc_model, double delta_t, size_t inter_step, size_t start_index, size_t end_index):
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_interpolatePose2MobileArmTraj(deref(opt_values.CValues_), Qc_model.CnoiseModel_Base_, delta_t, inter_step, start_index, end_index))
    return Values.cyCreateFromShared(ret)
def interpolatePose2Traj(Values opt_values, noiseModel_Base Qc_model, double delta_t, size_t inter_step, size_t start_index, size_t end_index):
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_interpolatePose2Traj(deref(opt_values.CValues_), Qc_model.CnoiseModel_Base_, delta_t, inter_step, start_index, end_index))
    return Values.cyCreateFromShared(ret)
def optimize(NonlinearFactorGraph graph, Values init_values, TrajOptimizerSetting setting, bool iter_no_increase):
    cdef shared_ptr[CValues] ret = make_shared[CValues](pxd_optimize(deref(graph.CNonlinearFactorGraph_), deref(init_values.CValues_), deref(setting.CTrajOptimizerSetting_), iter_no_increase))
    return Values.cyCreateFromShared(ret)

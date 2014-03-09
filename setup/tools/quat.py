# ***** BEGIN LICENSE BLOCK *****
# Version: MPL 1.1/GPL 2.0/LGPL 2.1
#
# The contents of this file are subject to the Mozilla Public License Version
# 1.1 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at
# http://www.mozilla.org/MPL/
#
# Software distributed under the License is distributed on an "AS IS" basis,
# WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
# for the specific language governing rights and limitations under the
# License.
#
# The Original Code is the Python Computer Graphics Kit.
#
# The Initial Developer of the Original Code is Matthias Baas.
# Portions created by the Initial Developer are Copyright (C) 2004
# the Initial Developer. All Rights Reserved.
#
# Contributor(s):
#
# Alternatively, the contents of this file may be used under the terms of
# either the GNU General Public License Version 2 or later (the "GPL"), or
# the GNU Lesser General Public License Version 2.1 or later (the "LGPL"),
# in which case the provisions of the GPL or the LGPL are applicable instead
# of those above. If you wish to allow use of your version of this file only
# under the terms of either the GPL or the LGPL, and not to allow others to
# use your version of this file under the terms of the MPL, indicate your
# decision by deleting the provisions above and replace them with the notice
# and other provisions required by the GPL or the LGPL. If you do not delete
# the provisions above, a recipient may use your version of this file under
# the terms of any one of the MPL, the GPL or the LGPL.
#
# ***** END LICENSE BLOCK *****
# $Id: quat.py,v 1.1 2005/08/15 15:39:48 mbaas Exp $

import types, math
from vec3 import vec3 as _vec3
from mat3 import mat3 as _mat3
from mat4 import mat4 as _mat4

# Comparison threshold
_epsilon = 1E-12

# quat
class quat:
    """Quaternion class.

    Quaternions are an extension to complex numbers and can be used
    to store rotations. They are composed of four floats which can be
    seen as an angle and an axis of rotation.
    """

    def __init__(self, *args):
        """Constructor.

        0 arguments: zeroes
        1 float argument:  w component, x,y,z = (0,0,0)
        1 quat argument: Make a copy
        1 mat3 argument: Initialize by rotation matrix
        1 mat4 argument: Initialize by rotation matrix
        2 arguments: angle & axis (doesn't have to be of unit length)
        4 arguments: components w,x,y,z
        """

        # 0 arguments
        if len(args)==0:
            self.w, self.x, self.y, self.z = (0.0, 0.0, 0.0, 0.0)

        # 1 arguments
        elif len(args)==1:
            T = type(args[0])
            # Scalar
            if T==types.FloatType or T==types.IntType or T==types.LongType:
                self.w = float(args[0])
                self.x, self.y, self.z = (0.0, 0.0, 0.0)
            # quat
            elif isinstance(args[0], quat):
                q=args[0]
                self.w = q.w
                self.x = q.x
                self.y = q.y
                self.z = q.z
            # mat3 or mat4
            elif isinstance(args[0], _mat3) or isinstance(args[0], _mat4):
                self.fromMat(args[0])
            # List or Tuple
            elif T==types.ListType or T==types.TupleType:
                dummy = quat(*args[0])
                self.w = dummy.w
                self.x = dummy.x
                self.y = dummy.y
                self.z = dummy.z                
            # String
            elif T==types.StringType:
                s=args[0].replace(","," ").replace("  "," ").strip().split(" ")
                if s==[""]:
                    s=[]
                f=map(lambda x: float(x), s)
                dummy = quat(f)
                self.w = dummy.w
                self.x = dummy.x
                self.y = dummy.y
                self.z = dummy.z
            else:
                raise TypeError, "quat() arg can't be converted to quat"

        # 2 arguments (angle & axis)
        elif len(args)==2:
            angle, axis = args
            self.fromAngleAxis(angle,axis)
            
        # 4 arguments
        elif len(args)==4:
            w,x,y,z = args
            self.w = float(w)
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

        else:
            raise TypeError, "quat() arg can't be converted to quat"
        

    def __repr__(self):
        return 'quat('+`self.w`+', '+`self.x`+', '+`self.y`+', '+`self.z`+')'

    def __str__(self):
        fmt="%1.4f"
        return '('+fmt%self.w+', '+fmt%self.x+', '+fmt%self.y+', '+fmt%self.z+')'

    def __eq__(self, other):
        """== operator

        >>> a=quat(1,2,3,4)
        >>> b=quat(6,7,8,9)
        >>> c=quat(6,7,8,9)
        >>> print a==b
        0
        >>> print b==c
        1
        >>> print a==None
        0
        """
        global _epsilon
        if isinstance(other, quat):
            return (abs(self.x-other.x)<=_epsilon and
                    abs(self.y-other.y)<=_epsilon and
                    abs(self.z-other.z)<=_epsilon and
                    abs(self.w-other.w)<=_epsilon)
        else:
            return False

    def __ne__(self, other):
        """!= operator

        >>> a=quat(1,2,3,4)
        >>> b=quat(6,7,8,9)
        >>> c=quat(6,7,8,9)
        >>> print a!=b
        1
        >>> print b!=c
        0
        >>> print a!=None
        1
        """
        return not (self==other)

    def __add__(self, other):
        """Addition.

        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print q+q
        (1.9378, 0.4320, 0.2160, 0.1080)
        """
        if isinstance(other, quat):
            return quat(self.w+other.w, self.x+other.x,
                        self.y+other.y, self.z+other.z)
        else:
            raise TypeError, "unsupported operand type for +"

    def __sub__(self, other):
        """Subtraction.

        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print q-q
        (0.0000, 0.0000, 0.0000, 0.0000)
        """
        if isinstance(other, quat):
            return quat(self.w-other.w, self.x-other.x,
                        self.y-other.y, self.z-other.z)
        else:
            raise TypeError, "unsupported operand type for +"

    def __mul__(self, other):
        """Multiplication.

        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print q*2.0
        (1.9378, 0.4320, 0.2160, 0.1080)
        >>> print 2.0*q
        (1.9378, 0.4320, 0.2160, 0.1080)
        >>> print q*q
        (0.8775, 0.4186, 0.2093, 0.1046)
        """
        T = type(other)
        # quat*scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return quat(self.w*other, self.x*other, self.y*other, self.z*other)
        # quat*quat
        if isinstance(other, quat):
            w1,x1,y1,z1 = self.w,self.x,self.y,self.z
            w2,x2,y2,z2 = other.w,other.x,other.y,other.z
            return quat(w1*w2-x1*x2-y1*y2-z1*z2,
                        w1*x2+x1*w2+y1*z2-z1*y2,
                        w1*y2+y1*w2-x1*z2+z1*x2,
                        w1*z2+z1*w2+x1*y2-y1*x2)
        # unsupported
        else:
            # Try to delegate the operation to the other operand
            if getattr(other,"__rmul__",None)!=None:
                return other.__rmul__(self)
            else:
                raise TypeError, "unsupported operand type for *"

    __rmul__ = __mul__

    def __div__(self, other):
        """Division.

        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print q/2.0
        (0.4844, 0.1080, 0.0540, 0.0270)
        """
        T = type(other)
        # quat/scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return quat(self.w/other, self.x/other, self.y/other, self.z/other)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for /"
        
    def __pow__(self, other):
        """Return self**q."""
#        if modulo!=None:
#            raise TypeError, "unsupported operation"
        q = quat(other)
        return (q*self.log()).exp()

    def __neg__(self):
        """Negation.

        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print -q
        (-0.9689, -0.2160, -0.1080, -0.0540)
        """
        return quat(-self.w, -self.x, -self.y, -self.z)

    def __pos__(self):
        """
        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print +q
        (0.9689, 0.2160, 0.1080, 0.0540)
        """
        return quat(+self.w, +self.x, +self.y, +self.z)

    def __abs__(self):
        """Return magnitude.
        
        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print round(abs(q),5)
        1.0
        """
        return math.sqrt(self.w*self.w + self.x*self.x +
                         self.y*self.y + self.z*self.z)

    def conjugate(self):
        """Return conjugate.
        
        >>> q=quat(0.9689, 0.2160, 0.1080, 0.0540)
        >>> print q.conjugate()
        (0.9689, -0.2160, -0.1080, -0.0540)
        """
        return quat(self.w, -self.x, -self.y, -self.z)

    def normalize(self):
        """Return normalized quaternion.

        >>> q=quat(0.9, 0.5, 0.2, 0.3)
        >>> q=q.normalize()
        >>> print q
        (0.8250, 0.4583, 0.1833, 0.2750)
        >>> print abs(q)
        1.0
        """
        nlen = 1.0/abs(self)
        return quat(self.w*nlen, self.x*nlen, self.y*nlen, self.z*nlen)

    def inverse(self):
        """Return inverse.

        >>> q=quat(0.9, 0.5, 0.2, 0.3)
        >>> print q.inverse()
        (0.7563, -0.4202, -0.1681, -0.2521)
        """
        len_2 = self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z
        return self.conjugate()/len_2

    def toAngleAxis(self):
        """Return angle (in radians) and rotation axis.

        >>> q=quat(0.9, 0.5, 0.2, 0.3)
        >>> angle, axis = q.toAngleAxis()
        >>> print round(angle,4)
        1.2014
        >>> print axis
        (0.8111, 0.3244, 0.4867)
        """

        nself = self.normalize()
        
        # Clamp nself.w (since the quat has to be normalized it should
        # be between -1 and 1 anyway, but it might be slightly off due
        # to numerical inaccuracies)
        w = max(min(nself.w,1.0),-1.0)
        
        w = math.acos(w)
        s = math.sin(w)
        if s<1E-12:
            return (0.0, _vec3(0.0,0.0,0.0))
        return (2.0*w, _vec3(nself.x/s, nself.y/s, nself.z/s))

    def fromAngleAxis(self, angle, axis):
        """Initialize self from an angle (in radians) and an axis and returns self."""
        if axis==_vec3(0):
            self.w = 1.0
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
        else:
            angle/=2.0
            self.w = math.cos(angle)
            x, y, z = axis
            s = math.sin(angle)/math.sqrt(x*x+y*y+z*z)
            self.x = x*s
            self.y = y*s
            self.z = z*s
            dummy = self.normalize()
            self.w = dummy.w
            self.x = dummy.x
            self.y = dummy.y
            self.z = dummy.z

        return self

    def toMat3(self):
        """Return rotation matrix as mat3."""
        x,y,z,w = self.x, self.y, self.z, self.w
        xx = 2.0*x*x
        yy = 2.0*y*y
        zz = 2.0*z*z
        xy = 2.0*x*y
        zw = 2.0*z*w
        xz = 2.0*x*z
        yw = 2.0*y*w
        yz = 2.0*y*z
        xw = 2.0*x*w
        return _mat3(1.0-yy-zz, xy-zw, xz+yw,
                     xy+zw, 1.0-xx-zz, yz-xw,
                     xz-yw, yz+xw, 1.0-xx-yy)

    def toMat4(self):
        """Return rotation matrix as mat4."""
        x,y,z,w = self.x, self.y, self.z, self.w
        xx = 2.0*x*x
        yy = 2.0*y*y
        zz = 2.0*z*z
        xy = 2.0*x*y
        zw = 2.0*z*w
        xz = 2.0*x*z
        yw = 2.0*y*w
        yz = 2.0*y*z
        xw = 2.0*x*w
        return _mat4(1.0-yy-zz, xy-zw, xz+yw, 0.0,
                     xy+zw, 1.0-xx-zz, yz-xw, 0.0,
                     xz-yw, yz+xw, 1.0-xx-yy, 0.0,
                     0.0, 0.0, 0.0, 1.0)

    def fromMat(self, m):
        """Initialize self from either a mat3 or mat4 and returns self."""
        global _epsilon
        
        d1,d2,d3 = m[0,0],m[1,1],m[2,2]
        t = d1+d2+d3+1.0
        if t>_epsilon:
            s = 0.5/math.sqrt(t)
            self.w = 0.25/s
            self.x = (m[2,1]-m[1,2])*s
            self.y = (m[0,2]-m[2,0])*s
            self.z = (m[1,0]-m[0,1])*s
        else:
            ad1 = d1
            ad2 = d2
            ad3 = d3
            if ad1>=ad2 and ad1>=ad3:
                s = math.sqrt(1.0+d1-d2-d3)*2.0
                self.x = 0.5/s
                self.y = (m[0,1]+m[1,0])/s
                self.z = (m[0,2]+m[2,0])/s
                self.w = (m[1,2]+m[2,1])/s
            elif ad2>=ad1 and ad2>=ad3:
                s = math.sqrt(1.0+d2-d1-d3)*2.0
                self.x = (m[0,1]+m[1,0])/s
                self.y = 0.5/s
                self.z = (m[1,2]+m[2,1])/s
                self.w = (m[0,2]+m[2,0])/s
            else:
                s = math.sqrt(1.0+d3-d1-d2)*2.0
                self.x = (m[0,2]+m[2,0])/s
                self.y = (m[1,2]+m[2,1])/s
                self.z = 0.5/s
                self.w = (m[0,1]+m[1,0])/s

        return self

    def dot(self, b):
        """Return the dot product of self and b."""
        return self.w*b.w + self.x*b.x + self.y*b.y + self.z*b.z

    def log(self):
        """Return the natural logarithm of self."""
        global _epsilon
        
        b = math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
        res = quat()
        if abs(b)<=_epsilon:
            res.x = 0.0
            res.y = 0.0
            res.z = 0.0
            if self.w<=_epsilon:
                raise ValueError, "math domain error"
            res.w = math.log(self.w)
        else:
            t = math.atan2(b, self.w)
            f = t/b
            res.x = f*self.x
            res.y = f*self.y
            res.z = f*self.z
            ct = math.cos(t)
            if abs(ct)<=_epsilon:
                raise ValueError, "math domain error"
            r = self.w/ct
            if r<=_epsilon:
                raise ValueError, "math domain error"            
            res.w = math.log(r)

        return res

    def exp(self):
        """Return the exponential of self."""
        global _epsilon
        
        b = math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
        res = quat()
        if abs(b)<=_epsilon:
            res.x = 0.0
            res.y = 0.0
            res.z = 0.0
            res.w = math.exp(self.w)
        else:
            f = math.sin(b)/b
            res.x = f*self.x
            res.y = f*self.y
            res.z = f*self.z
            res.w = math.exp(self.w)*math.cos(b)

        return res

    def rotateVec(self, v):
        """Return the rotated vector v.

        The quaternion must be a unit quaternion.
        This operation is equivalent to turning v into a quat, computing
        self*v*self.conjugate() and turning the result back into a vec3.
        """

        v = _vec3(v)
        ww = self.w*self.w
        xx = self.x*self.x
        yy = self.y*self.y
        zz = self.z*self.z
        wx = self.w*self.x
        wy = self.w*self.y
        wz = self.w*self.z
        xy = self.x*self.y
        xz = self.x*self.z
        yz = self.y*self.z

        return _vec3(ww*v.x + xx*v.x - yy*v.x - zz*v.x + 2*((xy-wz)*v.y + (xz+wy)*v.z),
                     ww*v.y - xx*v.y + yy*v.y - zz*v.y + 2*((xy+wz)*v.x + (yz-wx)*v.z),
                     ww*v.z - xx*v.z - yy*v.z + zz*v.z + 2*((xz-wy)*v.x + (yz+wx)*v.y))
    

def slerp(t, q0, q1, shortest=True):
    """Spherical linear interpolation between two quaternions.

    The return value is an interpolation between q0 and q1. For t=0.0
    the return value equals q0, for t=1.0 it equals q1.
    q0 and q1 must be unit quaternions.
    If shortest is True the interpolation is always done along the
    shortest path.
    """
    global _epsilon

    ca = q0.dot(q1)
    if shortest and ca<0:
        ca = -ca
        neg_q1 = True
    else:
        neg_q1 = False
    o = math.acos(ca)
    so = math.sin(o)

    if (abs(so)<=_epsilon):
        return quat(q0)

    a = math.sin(o*(1.0-t)) / so
    b = math.sin(o*t) / so
    if neg_q1:
        return q0*a - q1*b
    else:
        return q0*a + q1*b

def squad(t, a, b, c, d):
    """Spherical cubic interpolation."""
    return slerp(2*t*(1.0-t), slerp(t,a,d), slerp(t,b,c))


######################################################################

def _test():
    import doctest, quat
    failed, total = doctest.testmod(quat)
    print "%d/%d failed" % (failed, total)

if __name__=="__main__":

    _test()

#    q = quat(1.5,_vec3(1,0,0))
#    print q
#    m=q.toMat4().getMat3()
#    print m
#    w=quat(m)
#    print w



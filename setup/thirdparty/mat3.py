# coding: latin1
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
# $Id: mat3.py,v 1.2 2005/08/17 19:38:29 mbaas Exp $

import types, math, copy
from vec3 import vec3 as _vec3

# [  0   1   2 ]
# [  3   4   5 ]
# [  6   7   8 ]

# Comparison threshold
_epsilon = 1E-12


# mat3
class mat3:
    """Matrix class (3x3).

    This class represents a 3x3 matrix that can be used to store
    linear transformations.
    """

    def __init__(self, *args):
        """Constructor.

        There are several possibilities how to initialize a matrix,
        depending on the number of arguments you provide to the constructor.

        - 0 arguments: Every component is zero.
        - 1 number argument: The diagonal is initialized to that number,
          all the other elements are zero.
        - 1 sequence argument: The elements are initialized with the numbers
          in the sequence (the sequence must contain 9 numbers).
        - 1 mat3 argument: The matrix is copied.
        - 3 sequence arguments: The columns are initialized with the
          respective sequence (each sequence must contain 3 numbers).
        - 9 number arguments: The matrix is initialized with those values
          (row-major order).
        """

        # No arguments
        if len(args)==0:
            self.mlist = 9*[0.0]

        # 1 argument (list, scalar or mat3)
        elif len(args)==1:
            T = type(args[0])
            # Scalar
            if T==types.FloatType or T==types.IntType or T==types.LongType:
                f = float(args[0])
                self.mlist = [f,0.0,0.0,
                              0.0,f,0.0,
                              0.0,0.0,f]
            # mat3
            elif isinstance(args[0], mat3):
                self.mlist = copy.copy(args[0].mlist)
            # String
            elif T==types.StringType:
                s=args[0].replace(","," ").replace("  "," ").strip().split(" ")
                self.mlist=map(lambda x: float(x), s)
            else:
                self.mlist = mat3(*args[0]).mlist

        # 3 arguments (sequences)
        elif len(args)==3:
            a,b,c=args
            self.mlist = [a[0], b[0], c[0],
                          a[1], b[1], c[1],
                          a[2], b[2], c[2]]
            self.mlist = map(lambda x: float(x), self.mlist)

        # 9 arguments
        elif len(args)==9:
            self.mlist = map(lambda x: float(x), args)

        else:
            raise TypeError,"mat3() arg can't be converted to mat3"

        # Check if there are really 9 elements in the list
        if len(self.mlist)!=9:
            raise TypeError, "mat4(): Wrong number of matrix elements ("+`len(self.mlist)`+" instead of 9)"


    def __repr__(self):
        return 'mat3('+`self.mlist`[1:-1]+')'

    def __str__(self):
        fmt="%9.4f"
        m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
        return ('['+fmt%m11+', '+fmt%m12+', '+fmt%m13+']\n'+
                '['+fmt%m21+', '+fmt%m22+', '+fmt%m23+']\n'+
                '['+fmt%m31+', '+fmt%m32+', '+fmt%m33+']')

    def __eq__(self, other):
        """== operator"""
        global _epsilon
        if isinstance(other, mat3):
#            return self.mlist==other.mlist
            lst = filter(lambda (a,b): abs(a-b)>_epsilon, zip(self.mlist, other.mlist))
            return len(lst)==0
        else:
            return False

    def __ne__(self, other):
        """!= operator"""
        return not (self==other)


    def __add__(self, other):
        if isinstance(other, mat3):
            return mat3(map(lambda x,y: x+y, self.mlist, other.mlist))
        else:
            raise TypeError, "unsupported operand type for +"

    def __sub__(self, other):
        if isinstance(other, mat3):
            return mat3(map(lambda x,y: x-y, self.mlist, other.mlist))
        else:
            raise TypeError, "unsupported operand type for -"

    def __mul__(self, other):
        T = type(other)
        # mat3*scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat3(map(lambda x,other=other: x*other, self.mlist))
        # mat3*vec3
        if isinstance(other, _vec3):
            m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
            return _vec3(m11*other.x + m12*other.y + m13*other.z, 
                         m21*other.x + m22*other.y + m23*other.z, 
                         m31*other.x + m32*other.y + m33*other.z)            
        # mat3*mat3
        if isinstance(other, mat3):
            m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
            n11,n12,n13,n21,n22,n23,n31,n32,n33 = other.mlist
            return mat3( m11*n11+m12*n21+m13*n31,
                         m11*n12+m12*n22+m13*n32,
                         m11*n13+m12*n23+m13*n33,

                         m21*n11+m22*n21+m23*n31,
                         m21*n12+m22*n22+m23*n32,
                         m21*n13+m22*n23+m23*n33,

                         m31*n11+m32*n21+m33*n31,
                         m31*n12+m32*n22+m33*n32,
                         m31*n13+m32*n23+m33*n33)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for *"

    def __rmul__(self, other):
        T = type(other)
        # scalar*mat3
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat3(map(lambda x,other=other: other*x, self.mlist))
        # vec3*mat3
        if isinstance(other, _vec3):
            m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
            return _vec3(other.x*m11 + other.y*m21 + other.z*m31, 
                         other.x*m12 + other.y*m22 + other.z*m32, 
                         other.x*m13 + other.y*m23 + other.z*m33)
        # mat3*mat3
        if isinstance(other, mat3):
            return self.__mul__(other)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for *"


    def __div__(self, other):
        T = type(other)
        # mat3/scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat3(map(lambda x,other=other: x/other, self.mlist))
        # unsupported
        else:
            raise TypeError, "unsupported operand type for /"

    def __mod__(self, other):
        T = type(other)
        # mat3%scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat3(map(lambda x,other=other: x%other, self.mlist))
        # mat3%mat3
        if isinstance(other, mat3):
            return mat3(map(lambda (a,b): a%b, zip(self.mlist, other.mlist)))
        # unsupported
        else:
            raise TypeError, "unsupported operand type for %"


    def __neg__(self):
        return mat3(map(lambda x: -x, self.mlist))

    def __pos__(self):
        return mat3(map(lambda x: +x, self.mlist))


    def __len__(self):
        return 3

    def __getitem__(self, key):
        """Return a column or an individual element."""
        if type(key)==int:
            if   key==0:
                return _vec3(self.mlist[0],self.mlist[3],self.mlist[6])
            elif key==1:
                return _vec3(self.mlist[1],self.mlist[4],self.mlist[7])
            elif key==2:
                return _vec3(self.mlist[2],self.mlist[5],self.mlist[8])
            else:
                raise IndexError, "index out of range"                
        elif type(key)==types.TupleType:
            i,j=key
            if i<0 or i>2 or j<0 or j>2:
                raise IndexError, "index out of range"
            return self.mlist[i*3+j]
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def __setitem__(self, key, value):
        """Set a column or an individual element."""
        if type(key)==int:
            value = map(lambda x: float(x), value)
            if   key==0: self.mlist[0],self.mlist[3],self.mlist[6] = value
            elif key==1: self.mlist[1],self.mlist[4],self.mlist[7] = value
            elif key==2: self.mlist[2],self.mlist[5],self.mlist[8] = value
            else:
                raise IndexError, "index out of range"                
        elif type(key)==types.TupleType:
            i,j=key
            if i<0 or i>2 or j<0 or j>2:
                raise IndexError, "index out of range"
            self.mlist[i*3+j] = float(value)
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def getRow(self, index):
        """Return a row (as vec3)."""
        if type(index)==int:
            if index==0:
                return _vec3(self.mlist[0], self.mlist[1], self.mlist[2])
            elif index==1:
                return _vec3(self.mlist[3], self.mlist[4], self.mlist[5])
            elif index==2:
                return _vec3(self.mlist[6], self.mlist[7], self.mlist[8])
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be an integer"

    def setRow(self, index, value):
        """Set a row (as vec3)."""
        if type(index)==int:
            value = map(lambda x: float(x), value)
            if index==0: self.mlist[0], self.mlist[1], self.mlist[2] = value
            elif index==1: self.mlist[3], self.mlist[4], self.mlist[5] = value
            elif index==2: self.mlist[6], self.mlist[7], self.mlist[8] = value
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be an integer"

    def getColumn(self, index):
        """Return a column (as vec3)."""
        if type(index)==int:
            if index==0:
                return _vec3(self.mlist[0], self.mlist[3], self.mlist[6])
            elif index==1:
                return _vec3(self.mlist[1], self.mlist[4], self.mlist[7])
            elif index==2:
                return _vec3(self.mlist[2], self.mlist[5], self.mlist[8])
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be an integer"

    def setColumn(self, index, value):
        """Set a column."""
        if type(index)==int:
            value = map(lambda x: float(x), value)
            if index==0: self.mlist[0], self.mlist[3], self.mlist[6] = value
            elif index==1: self.mlist[1], self.mlist[4], self.mlist[7] = value
            elif index==2: self.mlist[2], self.mlist[5], self.mlist[8] = value
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be an integer"

    def getDiag(self):
        """Return the diagonal."""
        return _vec3(self.mlist[0], self.mlist[4], self.mlist[8])

    def setDiag(self, value):
        """Set diagonal."""
        a,b,c = value
        self.mlist[0] = float(a)
        self.mlist[4] = float(b)
        self.mlist[8] = float(c)

    def toList(self, rowmajor=0):
        """Create a list containing the matrix elements.

        By default the list is in column-major order. If you set the
        optional argument rowmajor to 1, you'll get the list in row-major
        order.
        """
        if rowmajor:
            return copy.copy(self.mlist)
        else:
            return self.transpose().mlist

    def identity():
        """Return the identity matrix."""
        return mat3(1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0)
    identity = staticmethod(identity)

    def transpose(self):
        """Return the transposed matrix."""
        m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
        return mat3(m11,m21,m31,
                    m12,m22,m32,
                    m13,m23,m33)

    def determinant(self):
        """Return determinant."""
        m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
        return m11*m22*m33+ \
               m12*m23*m31+ \
               m13*m21*m32- \
               m31*m22*m13- \
               m32*m23*m11- \
               m33*m21*m12

    def inverse(self):
        """Return inverse matrix."""
        m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist
        d = 1.0/self.determinant()
        return mat3( m22*m33-m23*m32, m32*m13-m12*m33, m12*m23-m22*m13,
                     m23*m31-m21*m33, m11*m33-m31*m13, m21*m13-m11*m23,
                     m21*m32-m31*m22, m31*m12-m11*m32, m11*m22-m12*m21 )*d

    def scaling(s):
        """Return a scale transformation."""
        sx,sy,sz = s
        return mat3(sx, 0.0, 0.0,
                    0.0, sy, 0.0,
                    0.0, 0.0, sz)
    scaling = staticmethod(scaling)

    def rotation(angle, axis):
        """Return a rotation matrix."""
        axis = _vec3(axis)
        
        sqr_a = axis.x*axis.x
        sqr_b = axis.y*axis.y
        sqr_c = axis.z*axis.z
        len2  = sqr_a+sqr_b+sqr_c

        k2    = math.cos(angle)
        k1    = (1.0-k2)/len2
        k3    = math.sin(angle)/math.sqrt(len2)
        k1ab  = k1*axis.x*axis.y
        k1ac  = k1*axis.x*axis.z
        k1bc  = k1*axis.y*axis.z
        k3a   = k3*axis.x
        k3b   = k3*axis.y
        k3c   = k3*axis.z

        return mat3( k1*sqr_a+k2, k1ab-k3c, k1ac+k3b,
                     k1ab+k3c, k1*sqr_b+k2, k1bc-k3a,
                     k1ac-k3b, k1bc+k3a, k1*sqr_c+k2)
    rotation = staticmethod(rotation)

    def scale(self, s):
        sx = float(s[0])
        sy = float(s[1])
        sz = float(s[2])
        self.mlist[0] *= sx
        self.mlist[1] *= sy
        self.mlist[2] *= sz
        self.mlist[3] *= sx
        self.mlist[4] *= sy
        self.mlist[5] *= sz
        self.mlist[6] *= sx
        self.mlist[7] *= sy
        self.mlist[8] *= sz
        return self

    def rotate(self, angle, axis):
        R=self.rotation(angle, axis)
        self.mlist = (self*R).mlist
        return self

    def ortho(self):
        """Return a matrix with orthogonal base vectors.
        """

        m11,m12,m13,m21,m22,m23,m31,m32,m33 = self.mlist

        x = _vec3(m11, m21, m31)
        y = _vec3(m12, m22, m32)
        z = _vec3(m13, m23, m33)

        xl = x.length()
        xl*=xl
        y = y - ((x*y)/xl)*x
        z = z - ((x*z)/xl)*x

        yl = y.length()
        yl*=yl
        z = z - ((y*z)/yl)*y

        return mat3( x.x, y.x, z.x,
                     x.y, y.y, z.y,
                     x.z, y.z, z.z)

    def decompose(self):
        """Decomposes the matrix into a rotation and scaling part.

        Returns a tuple (rotation, scaling). The scaling part is given
        as a vec3, the rotation is still a mat3.
        """
        dummy = self.ortho()

        x = dummy.getColumn(0)
        y = dummy.getColumn(1)
        z = dummy.getColumn(2)
        xl = x.length()
        yl = y.length()
        zl = z.length()
        scale = _vec3(xl,yl,zl)
        
        x/=xl
        y/=yl
        z/=zl
        dummy.setColumn(0,x)
        dummy.setColumn(1,y)
        dummy.setColumn(2,z)
        if dummy.determinant()<0.0:
            dummy.setColumn(0,-x)
            scale.x=-scale.x

        return (dummy, scale)

    def fromEulerYXZ(x, y, z):
        """Initializes self from Euler angles."""
        A = math.cos(x)
        B = math.sin(x)
        C = math.cos(y)
        D = math.sin(y)
        E = math.cos(z)
        F = math.sin(z)
        CE = C*E
        CF = C*F
        DE = D*E
        DF = D*F

        return mat3( CE+DF*B, DE*B-CF, A*D,
                     A*F, A*E, -B,
                     CF*B-DE, DF+CE*B, A*C )

    fromEulerYXZ = staticmethod(fromEulerYXZ)

    def fromEulerZXY(x, y, z):
        """Initializes self from Euler angles."""
        A = math.cos(x)
        B = math.sin(x)
        C = math.cos(y)
        D = math.sin(y)
        E = math.cos(z)
        F = math.sin(z)
        CE = C*E
        CF = C*F
        DE = D*E
        DF = D*F

        return mat3( CE-DF*B, -A*F, DE+CF*B,
                     CF+DE*B, A*E, DF-CE*B,
                     -A*D, B, A*C )

    fromEulerZXY = staticmethod(fromEulerZXY)

    def fromEulerZYX(x, y, z):
        """Initializes self from Euler angles."""
        A = math.cos(x)
        B = math.sin(x)
        C = math.cos(y)
        D = math.sin(y)
        E = math.cos(z)
        F = math.sin(z)
        AE = A*E
        AF = A*F
        BE = B*E
        BF = B*F

        return mat3( C*E, BE*D-AF, AE*D+BF,
                     C*F, BF*D+AE, AF*D-BE,
                     -D, B*C, A*C )

    fromEulerZYX = staticmethod(fromEulerZYX)

    def fromEulerYZX(x, y, z):
        """Initializes self from Euler angles."""
        A = math.cos(x)
        B = math.sin(x)
        C = math.cos(y)
        D = math.sin(y)
        E = math.cos(z)
        F = math.sin(z)
        AC = A*C
        AD = A*D
        BC = B*C
        BD = B*D

        return mat3( C*E, BD-AC*F, BC*F+AD,
                     F, A*E, -B*E,
                     -D*E, AD*F+BC, AC-BD*F )

    fromEulerYZX = staticmethod(fromEulerYZX)

    def fromEulerXZY(x, y, z):
        """Initializes self from Euler angles."""
        A = math.cos(x)
        B = math.sin(x)
        C = math.cos(y)
        D = math.sin(y)
        E = math.cos(z)
        F = math.sin(z)
        AC = A*C
        AD = A*D
        BC = B*C
        BD = B*D

        return mat3( C*E, -F, D*E,
                     AC*F+BD, A*E, AD*F-BC,
                     BC*F-AD, B*E, BD*F+AC )

    fromEulerXZY = staticmethod(fromEulerXZY)

    def fromEulerXYZ(x, y, z):
        """Initializes self from Euler angles."""
        A = math.cos(x)
        B = math.sin(x)
        C = math.cos(y)
        D = math.sin(y)
        E = math.cos(z)
        F = math.sin(z)
        AE = A*E
        AF = A*F
        BE = B*E
        BF = B*F

        return mat3( C*E, -C*F, D,
                     AF+BE*D, AE-BF*D, -B*C,
                     BF-AE*D, BE+AF*D, A*C )

    fromEulerXYZ = staticmethod(fromEulerXYZ)

    def toEulerYXZ(self):
        """Return the Euler angles of a rotation matrix."""
        y,x,z = self._getRotation(2, True, True, True)
        return (x,y,z)

    def toEulerZXY(self):
        """Return the Euler angles of a rotation matrix."""
        z,x,y = self._getRotation(1, False, True, True)
        return (x,y,z)

    def toEulerZYX(self):
        """Return the Euler angles of a rotation matrix."""
        z,y,x = self._getRotation(0, True, True, True)
        return (x,y,z)

    def toEulerYZX(self):
        """Return the Euler angles of a rotation matrix."""
        y,z,x = self._getRotation(0, False, True, True)
        return (x,y,z)

    def toEulerXZY(self):
        """Return the Euler angles of a rotation matrix."""
        x,z,y = self._getRotation(1, True, True, True)
        return (x,y,z)

    def toEulerXYZ(self):
        """Return the Euler angles of a rotation matrix."""
        x,y,z = self._getRotation(2, False, True, True)
        return (x,y,z)

    def fromToRotation(_from, to):
        """Returns a rotation matrix that rotates one vector into another.

        The generated rotation matrix will rotate the vector _from into
        the vector to. _from and to must be unit vectors!

        This method is based on the code from:

        Tomas Möller, John Hughes
        Efficiently Building a Matrix to Rotate One Vector to Another
        Journal of Graphics Tools, 4(4):1-4, 1999
        http://www.acm.org/jgt/papers/MollerHughes99/
        """
        
        _from = _vec3(_from)
        to = _vec3(to)
        EPSILON = 0.000001
        e = _from*to
        f = abs(e)

        if (f>1.0-EPSILON):    # "from" and "to"-vector almost parallel
            # vector most nearly orthogonal to "from"
            fx = abs(_from.x)
            fy = abs(_from.y)
            fz = abs(_from.z)

            if (fx<fy):
                if (fx<fz):
                    x = _vec3(1.0, 0.0, 0.0)
                else:
                    x = _vec3(0.0, 0.0, 1.0)
            else:
                if (fy<fz):
                    x = _vec3(0.0, 1.0, 0.0)
                else:
                    x = _vec3(0.0, 0.0, 1.0)

            u = x-_from
            v = x-to

            c1 = 2.0/(u*u)
            c2 = 2.0/(v*v)
            c3 = c1*c2*u*v

            res = mat3()
            for i in range(3):
                for j in range(3):
                    res[i,j] =  - c1*u[i]*u[j] - c2*v[i]*v[j] + c3*v[i]*u[j]
                res[i,i] += 1.0
                
            return res
                
        else:  # the most common case, unless "from"="to", or "from"=-"to"
            v = _from.cross(to)
            h = 1.0/(1.0 + e)    # optimization by Gottfried Chen
            hvx = h*v.x
            hvz = h*v.z
            hvxy = hvx*v.y
            hvxz = hvx*v.z
            hvyz = hvz*v.y

            m11 = e + hvx*v.x
            m12 = hvxy - v.z
            m13 = hvxz + v.y

            m21 = hvxy + v.z
            m22 = e + h*v.y*v.y
            m23 = hvyz - v.x

            m31 = hvxz - v.y
            m32 = hvyz + v.x
            m33 = e + hvz*v.z

            return mat3(m11,m12,m13,m21,m22,m23,m31,m32,m33)

    fromToRotation = staticmethod(fromToRotation)


    def _getRotation(self, i, neg, alt, rev):
        """Get Euler angles in any of the 24 different conventions.
    
        The first four argument select a particular convention. The last three
        output arguments receive the angles. The order of the angles depends
        on the convention.
    
        See http://www.cgafaq.info/wiki/Euler_angles_from_matrix for the
        algorithm used.
    
        i: The index of the first axis (global rotations, s) or last axis (local rotations, r). 0=XZX, 1=YXY, 2=ZYZ
        neg: If true, the convention contains an odd permutation of the convention defined by i alone (i.e. the middle axis is replaced. For example, XZX -> XYX)
        alt: If true, the first and last axes are different. Local rotations: The first axis changes. For example, XZX -> YZX
        rev: If true, the first and last angle are exchanged. This toggles between global/local rotations. In all the concrete getRotation*() functions this is always true because all the functions assume local rotations.
        """    
        v = [self[0,i], self[1,i], self[2,i]]

        j,k,h = _eulerIndices(i, neg, alt)

        a = v[h]
        b = v[k]
        c,s,r = _eulerGivens(a, b)
        v[h] = r
        s1 = c*self[k,j] - s*self[h,j]
        c1 = c*self[k,k] - s*self[h,k]
        r1 = math.atan2(s1, c1)
        r2 = math.atan2(v[j], v[i])
        r3 = math.atan2(s, c)
        if alt:
            r3 = -r3
        if neg:
            r1 = -r1
            r2 = -r2
            r3 = -r3
        if rev:
            tmp = r1
            r1 = r3
            r3 = tmp
        return r1,r2,r3

def _eulerIndices(i, neg, alt):
    """Helper function for _getRotation()."""
    next = [1, 2, 0, 1]
    j = next[i+int(neg)]
    k = 3-i-j
    h = next[k+(1^int(neg)^int(alt))]
    return j,k,h

def _eulerGivens(a, b):
    """Helper function for _getRotation()."""
    global _epsilon
    
    absa = abs(a)
    absb = abs(b)
    # b=0?
    if absb<=_epsilon:
        if a>=0:
            c = 1.0
        else:
            c = -1.0
        return (c, 0.0, absa)
    # a=0?
    elif absa<=_epsilon:
        if b>=0:
            s = 1.0
        else:
            s = -1.0
        return (0.0, s, absb)
    # General case
    else:
        if absb>absa:
            t = a/b
            u = math.sqrt(1.0+t*t)
            if b<0:
                u = -u
            s = 1.0/u
            c = s*t
            r = b*u
        else:
            t = b/a
            u = math.sqrt(1.0+t*t)
            if (a<0):
                u = -u
            c = 1.0/u
            s = c*t
            r = a*u
        return c,s,r

######################################################################

if __name__=="__main__":

    vec3 = _vec3
    a=vec3(1,2,3)

    M = mat3("2,4,5,6")

    a=mat3(M)
    a[0,0]=17
    print M
    print a

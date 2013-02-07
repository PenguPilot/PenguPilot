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
# $Id: mat4.py,v 1.2 2005/08/17 19:52:41 mbaas Exp $

import types, math, copy
from vec3 import vec3 as _vec3
from vec4 import vec4 as _vec4
from mat3 import mat3 as _mat3


# [  0   1   2   3 ]
# [  4   5   6   7 ]
# [  8   9  10  11 ]
# [ 12  13  14  15 ]

# Comparison threshold
_epsilon = 1E-12


# mat4
class mat4:
    """Matrix class (4x4).

    This class represents a 4x4 matrix that can be used to store
    affine transformations.
    """

    def __init__(self, *args):
        "Constructor"

        # No arguments
        if len(args)==0:
            self.mlist = 16*[0.0]

        # 1 argument (list, scalar or mat4)
        elif len(args)==1:
            T = type(args[0])
            if T==types.FloatType or T==types.IntType or T==types.LongType:
                f = float(args[0])
                self.mlist = [f,0.0,0.0,0.0,
                              0.0,f,0.0,0.0,
                              0.0,0.0,f,0.0,
                              0.0,0.0,0.0,f]
            # mat4
            elif isinstance(args[0], mat4):
                self.mlist = copy.copy(args[0].mlist)
            # String
            elif T==types.StringType:
                s=args[0].replace(","," ").replace("  "," ").strip().split(" ")
                self.mlist=map(lambda x: float(x), s)
            else:
                self.mlist = mat4(*args[0]).mlist

        # 4 arguments (sequences)
        elif len(args)==4:
            a,b,c,d=args
            self.mlist = [a[0], b[0], c[0], d[0],
                          a[1], b[1], c[1], d[1],
                          a[2], b[2], c[2], d[2],
                          a[3], b[3], c[3], d[3]]
            self.mlist = map(lambda x: float(x), self.mlist)

        # 16 arguments
        elif len(args)==16:
            self.mlist = map(lambda x: float(x), args)

        else:
            raise TypeError,"mat4() arg can't be converted to mat4"

        # Check if there are really 16 elements in the list
        if len(self.mlist)!=16:
            raise TypeError, "mat4(): Wrong number of matrix elements ("+`len(self.mlist)`+" instead of 16)"

    def __repr__(self):
        return 'mat4('+`self.mlist`[1:-1]+')'

    def __str__(self):
        fmt="%9.4f"
        m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
        return ('['+fmt%m11+', '+fmt%m12+', '+fmt%m13+', '+fmt%m14+']\n'+
                '['+fmt%m21+', '+fmt%m22+', '+fmt%m23+', '+fmt%m24+']\n'+
                '['+fmt%m31+', '+fmt%m32+', '+fmt%m33+', '+fmt%m34+']\n'+
                '['+fmt%m41+', '+fmt%m42+', '+fmt%m43+', '+fmt%m44+']')

    def __eq__(self, other):
        """== operator"""
        global _epsilon
        if isinstance(other, mat4):
#            return self.mlist==other.mlist
            lst = filter(lambda (a,b): abs(a-b)>_epsilon, zip(self.mlist, other.mlist))
            return len(lst)==0
        else:
            return False

    def __ne__(self, other):
        """!= operator"""
        return not (self==other)

    def __add__(self, other):
        """Matrix addition.

        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M+M
        [   2.0000,    4.0000,    6.0000,    8.0000]
        [  10.0000,   12.0000,   14.0000,   16.0000]
        [  18.0000,   20.0000,   22.0000,   24.0000]
        [  26.0000,   28.0000,   30.0000,   32.0000]
        """
        if isinstance(other, mat4):
            return mat4(map(lambda x,y: x+y, self.mlist, other.mlist))
        else:
            raise TypeError, "unsupported operand type for +"

    def __sub__(self, other):
        """Matrix subtraction.

        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M-M
        [   0.0000,    0.0000,    0.0000,    0.0000]
        [   0.0000,    0.0000,    0.0000,    0.0000]
        [   0.0000,    0.0000,    0.0000,    0.0000]
        [   0.0000,    0.0000,    0.0000,    0.0000]
        """
        if isinstance(other, mat4):
            return mat4(map(lambda x,y: x-y, self.mlist, other.mlist))
        else:
            raise TypeError, "unsupported operand type for -"

    def __mul__(self, other):
        """Multiplication.

        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M*2.0
        [   2.0000,    4.0000,    6.0000,    8.0000]
        [  10.0000,   12.0000,   14.0000,   16.0000]
        [  18.0000,   20.0000,   22.0000,   24.0000]
        [  26.0000,   28.0000,   30.0000,   32.0000]
        >>> print 2.0*M
        [   2.0000,    4.0000,    6.0000,    8.0000]
        [  10.0000,   12.0000,   14.0000,   16.0000]
        [  18.0000,   20.0000,   22.0000,   24.0000]
        [  26.0000,   28.0000,   30.0000,   32.0000]
        >>> print M*M
        [  90.0000,  100.0000,  110.0000,  120.0000]
        [ 202.0000,  228.0000,  254.0000,  280.0000]
        [ 314.0000,  356.0000,  398.0000,  440.0000]
        [ 426.0000,  484.0000,  542.0000,  600.0000]
        >>> print M*_vec3(1,2,3)
        (0.1765, 0.4510, 0.7255)
        >>> print _vec3(1,2,3)*M
        (0.7083, 0.8056, 0.9028)
        """
        T = type(other)
        # mat4*scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat4(map(lambda x,other=other: x*other, self.mlist))
        # mat4*vec3
        if isinstance(other, _vec3):
            m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
            w = float(m41*other.x + m42*other.y + m43*other.z + m44)
            return _vec3(m11*other.x + m12*other.y + m13*other.z + m14, 
                         m21*other.x + m22*other.y + m23*other.z + m24, 
                         m31*other.x + m32*other.y + m33*other.z + m34)/w
        # mat4*vec4
        if isinstance(other, _vec4):
            m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
            return _vec4(m11*other.x + m12*other.y + m13*other.z + m14*other.w, 
                         m21*other.x + m22*other.y + m23*other.z + m24*other.w, 
                         m31*other.x + m32*other.y + m33*other.z + m34*other.w,
                         m41*other.x + m42*other.y + m43*other.z + m44*other.w)
        # mat4*mat4
        if isinstance(other, mat4):
            m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
            n11,n12,n13,n14,n21,n22,n23,n24,n31,n32,n33,n34,n41,n42,n43,n44 = other.mlist
            return mat4( m11*n11+m12*n21+m13*n31+m14*n41,
                         m11*n12+m12*n22+m13*n32+m14*n42,
                         m11*n13+m12*n23+m13*n33+m14*n43,
                         m11*n14+m12*n24+m13*n34+m14*n44,

                         m21*n11+m22*n21+m23*n31+m24*n41,
                         m21*n12+m22*n22+m23*n32+m24*n42,
                         m21*n13+m22*n23+m23*n33+m24*n43,
                         m21*n14+m22*n24+m23*n34+m24*n44,

                         m31*n11+m32*n21+m33*n31+m34*n41,
                         m31*n12+m32*n22+m33*n32+m34*n42,
                         m31*n13+m32*n23+m33*n33+m34*n43,
                         m31*n14+m32*n24+m33*n34+m34*n44,

                         m41*n11+m42*n21+m43*n31+m44*n41,
                         m41*n12+m42*n22+m43*n32+m44*n42,
                         m41*n13+m42*n23+m43*n33+m44*n43,
                         m41*n14+m42*n24+m43*n34+m44*n44)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for *"

    def __rmul__(self, other):
        T = type(other)
        # scalar*mat4
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat4(map(lambda x,other=other: other*x, self.mlist))
        # vec4*mat4
        if isinstance(other, _vec4):
            m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
            return _vec4(other.x*m11 + other.y*m21 + other.z*m31 + other.w*m41, 
                         other.x*m12 + other.y*m22 + other.z*m32 + other.w*m42,
                         other.x*m13 + other.y*m23 + other.z*m33 + other.w*m43,
                         other.x*m14 + other.y*m24 + other.z*m34 + other.w*m44)
        # vec3*mat4
        if isinstance(other, _vec3):
            m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
            w = float(other.x*m14 + other.y*m24 + other.z*m34 + m44)
            return _vec3(other.x*m11 + other.y*m21 + other.z*m31 + m41, 
                         other.x*m12 + other.y*m22 + other.z*m32 + m42,
                         other.x*m13 + other.y*m23 + other.z*m33 + m43)/w
        # mat4*mat4
        if isinstance(other, mat4):
            return self.__mul__(other)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for *"

    def __div__(self, other):
        """Division
        
        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M/2.0
        [   0.5000,    1.0000,    1.5000,    2.0000]
        [   2.5000,    3.0000,    3.5000,    4.0000]
        [   4.5000,    5.0000,    5.5000,    6.0000]
        [   6.5000,    7.0000,    7.5000,    8.0000]
        """
        T = type(other)
        # mat4/scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat4(map(lambda x,other=other: x/other, self.mlist))
        # unsupported
        else:
            raise TypeError, "unsupported operand type for /"

    def __mod__(self, other):
        """Modulo.

        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M%5.0
        [   1.0000,    2.0000,    3.0000,    4.0000]
        [   0.0000,    1.0000,    2.0000,    3.0000]
        [   4.0000,    0.0000,    1.0000,    2.0000]
        [   3.0000,    4.0000,    0.0000,    1.0000]
        """
        T = type(other)
        # mat4%scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return mat4(map(lambda x,other=other: x%other, self.mlist))
        # mat4%mat4
        if isinstance(other, mat4):
            return mat4(map(lambda (a,b): a%b, zip(self.mlist, other.mlist)))
        # unsupported
        else:
            raise TypeError, "unsupported operand type for %"

    def __neg__(self):
        """Negation.

        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print -M
        [  -1.0000,   -2.0000,   -3.0000,   -4.0000]
        [  -5.0000,   -6.0000,   -7.0000,   -8.0000]
        [  -9.0000,  -10.0000,  -11.0000,  -12.0000]
        [ -13.0000,  -14.0000,  -15.0000,  -16.0000]
        """
        return mat4(map(lambda x: -x, self.mlist))

    def __pos__(self):
        """
        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print +M
        [   1.0000,    2.0000,    3.0000,    4.0000]
        [   5.0000,    6.0000,    7.0000,    8.0000]
        [   9.0000,   10.0000,   11.0000,   12.0000]
        [  13.0000,   14.0000,   15.0000,   16.0000]
        """
        return mat4(map(lambda x: +x, self.mlist))


    def __len__(self):
        return 4

    def __getitem__(self, key):
        if type(key)==types.IntType:
            if key<0 or key>3:
                raise IndexError,"index out of range"
            m=self.mlist
            if   key==0: return _vec4(m[0],m[4],m[8],m[12])
            elif key==1: return _vec4(m[1],m[5],m[9],m[13])
            elif key==2: return _vec4(m[2],m[6],m[10],m[14])
            elif key==3: return _vec4(m[3],m[7],m[11],m[15])
        elif type(key)==types.TupleType:
            i,j=key
            if i<0 or i>3 or j<0 or j>3:
                raise IndexError, "index out of range"
            return self.mlist[i*4+j]
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def __setitem__(self, key, value):
        if type(key)==types.IntType:
            if key<0 or key>3:
                raise IndexError,"index out of range"
            m=self.mlist
            value = map(lambda x: float(x), value)
            if   key==0: m[0],m[4],m[8],m[12]=value
            elif key==1: m[1],m[5],m[9],m[13]=value
            elif key==2: m[2],m[6],m[10],m[14]=value
            elif key==3: m[3],m[7],m[11],m[15]=value
        elif type(key)==types.TupleType:
            i,j=key
            if i<0 or i>3 or j<0 or j>3:
                raise IndexError, "index out of range"
            self.mlist[i*4+j] = float(value)
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def getRow(self, index):
        """Return row (as vec4)."""
        if type(index)==int:
            m=self.mlist
            if   index==0: return _vec4(m[0], m[1], m[2], m[3])
            elif index==1: return _vec4(m[4], m[5], m[6], m[7])
            elif index==2: return _vec4(m[8], m[9], m[10], m[11])
            elif index==3: return _vec4(m[12], m[13], m[14], m[15])
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def setRow(self, index, value):
        """Set row."""
        if type(index)==int:
            m=self.mlist
            value = map(lambda x: float(x), value)
            if   index==0: m[0],m[1],m[2],m[3] = value
            elif index==1: m[4],m[5],m[6],m[7] = value
            elif index==2: m[8],m[9],m[10],m[11] = value
            elif index==3: m[12],m[13],m[14],m[15] = value
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def getColumn(self, index):
        """Return column (as vec4)."""
        if type(index)==int:
            m=self.mlist
            if   index==0: return _vec4(m[0], m[4], m[8], m[12])
            elif index==1: return _vec4(m[1], m[5], m[9], m[13])
            elif index==2: return _vec4(m[2], m[6], m[10], m[14])
            elif index==3: return _vec4(m[3], m[7], m[11], m[15])
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def setColumn(self, index, value):
        """Set column."""
        if type(index)==int:
            m = self.mlist
            value = map(lambda x: float(x), value)
            if   index==0: m[0],m[4],m[8],m[12] = value
            elif index==1: m[1],m[5],m[9],m[13] = value
            elif index==2: m[2],m[6],m[10],m[14] = value
            elif index==3: m[3],m[7],m[11],m[15] = value
            else:
                raise IndexError,"index out of range"
        else:
            raise TypeError,"index must be integer or 2-tuple"

    def getDiag(self):
        """Return the diagonal."""
        return _vec4(self.mlist[0], self.mlist[5], self.mlist[10], self.mlist[15])

    def setDiag(self, value):
        """Set diagonal."""
        a,b,c,d = value
        self.mlist[0] = float(a)
        self.mlist[5] = float(b)
        self.mlist[10] = float(c)
        self.mlist[15] = float(d)

    def toList(self, rowmajor=0):
        """Return a list containing the matrix elements.

        By default the list is in column-major order (which can directly be
        used in OpenGL or RenderMan). If you set the optional argument
        rowmajor to 1, you'll get the list in row-major order.

        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M.toList()
        [1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15, 4, 8, 12, 16]
        >>> print M.toList(rowmajor=1)
        [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        """
        if rowmajor:
            return copy.copy(self.mlist)
        else:
            return self.transpose().mlist
            

    def identity():
        """Return identity matrix.

        >>> print mat4().identity()
        [   1.0000,    0.0000,    0.0000,    0.0000]
        [   0.0000,    1.0000,    0.0000,    0.0000]
        [   0.0000,    0.0000,    1.0000,    0.0000]
        [   0.0000,    0.0000,    0.0000,    1.0000]
        """
        return mat4(1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0)
    identity = staticmethod(identity)

    def transpose(self):
        """Transpose matrix.
        
        >>> M=mat4(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16)
        >>> print M.transpose()
        [   1.0000,    5.0000,    9.0000,   13.0000]
        [   2.0000,    6.0000,   10.0000,   14.0000]
        [   3.0000,    7.0000,   11.0000,   15.0000]
        [   4.0000,    8.0000,   12.0000,   16.0000]
        """
        m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
        return mat4(m11,m21,m31,m41,
                    m12,m22,m32,m42,
                    m13,m23,m33,m43,
                    m14,m24,m34,m44)

    def determinant(self):
        """Return determinant.
        
        >>> M=mat4(2.0,0,0,0, 0,2.0,0,0, 0,0,2.0,0, 0,0,0,2.0)
        >>> print M.determinant()
        16.0
        """
        m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist

        return m11*m22*m33*m44 \
               -m11*m22*m34*m43 \
               +m11*m23*m34*m42 \
               -m11*m23*m32*m44 \
               +m11*m24*m32*m43 \
               -m11*m24*m33*m42 \
               -m12*m23*m34*m41 \
               +m12*m23*m31*m44 \
               -m12*m24*m31*m43 \
               +m12*m24*m33*m41 \
               -m12*m21*m33*m44 \
               +m12*m21*m34*m43 \
               +m13*m24*m31*m42 \
               -m13*m24*m32*m41 \
               +m13*m21*m32*m44 \
               -m13*m21*m34*m42 \
               +m13*m22*m34*m41 \
               -m13*m22*m31*m44 \
               -m14*m21*m32*m43 \
               +m14*m21*m33*m42 \
               -m14*m22*m33*m41 \
               +m14*m22*m31*m43 \
               -m14*m23*m31*m42 \
               +m14*m23*m32*m41
    

    def _submat(self, i,j):
        M=_mat3()
        for k in range(3):
            for l in range(3):
                t=(k,l)
                if k>=i:
                    t=(k+1,t[1])
                if l>=j:
                    t=(t[0],l+1)
                M[k,l] = self[t]
        return M
        
    def inverse(self):
        """Return inverse matrix.

        >>> M=mat4(0,-2.0,0,0, 2.0,0,0,0, 0,0,2,0, 0,0,0,2)
        >>> print M.inverse()
        [   0.0000,    0.5000,    0.0000,    0.0000]
        [  -0.5000,    0.0000,    0.0000,    0.0000]
        [   0.0000,    0.0000,    0.5000,    0.0000]
        [   0.0000,    0.0000,    0.0000,    0.5000]
        """
        
        Mi=mat4()
        d=self.determinant()
        for i in range(4):
            for j in range(4):
                sign=1-((i+j)%2)*2
                m3=self._submat(i,j)
                Mi[j,i]=sign*m3.determinant()/d
        return Mi

    def translation(t):
        """Return translation matrix."""
        tx,ty,tz = t
        return mat4(1.0, 0.0, 0.0, tx,
                    0.0, 1.0, 0.0, ty,
                    0.0, 0.0, 1.0, tz,
                    0.0, 0.0, 0.0, 1.0)
    translation = staticmethod(translation)

    def scaling(s):
        """Return scaling matrix."""
        sx,sy,sz = s
        return mat4(sx, 0.0, 0.0, 0.0,
                    0.0, sy, 0.0, 0.0,
                    0.0, 0.0, sz, 0.0,
                    0.0, 0.0, 0.0, 1.0)
    scaling = staticmethod(scaling)

    def rotation(angle, axis):
        """Return rotation matrix.

        angle must be given in radians. axis should be of type vec3.
        """
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

        return mat4( k1*sqr_a+k2, k1ab-k3c, k1ac+k3b, 0.0,
                     k1ab+k3c, k1*sqr_b+k2, k1bc-k3a, 0.0,
                     k1ac-k3b, k1bc+k3a, k1*sqr_c+k2, 0.0,
                     0.0, 0.0, 0.0, 1.0)
    rotation = staticmethod(rotation)

    def translate(self, t):
        """Concatenate a translation."""
        tx = float(t[0])
        ty = float(t[1])
        tz = float(t[2])
        m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
        self.mlist[3]  = m11*tx + m12*ty + m13*tz + m14
        self.mlist[7]  = m21*tx + m22*ty + m23*tz + m24
        self.mlist[11] = m31*tx + m32*ty + m33*tz + m34
        self.mlist[15] = m41*tx + m42*ty + m43*tz + m44
        return self

    def scale(self, s):
        """Concatenate a scaling."""
        sx = float(s[0])
        sy = float(s[1])
        sz = float(s[2])
        self.mlist[0]  *= sx
        self.mlist[1]  *= sy
        self.mlist[2]  *= sz
        self.mlist[4]  *= sx
        self.mlist[5]  *= sy
        self.mlist[6]  *= sz
        self.mlist[8]  *= sx
        self.mlist[9]  *= sy
        self.mlist[10] *= sz
        self.mlist[12] *= sx
        self.mlist[13] *= sy
        self.mlist[14] *= sz
        return self

    def rotate(self, angle, axis):
        """Concatenate a rotation.

        angle must be given in radians. axis should be of type vec3.
        """
        R=self.rotation(angle, axis)
        self.mlist = (self*R).mlist
        return self


    def frustum(left, right, bottom, top, near, far):
        """Set a perspective transformation.
        
        This method is equivalent to the OpenGL command glFrustum().
        """
        return mat4( (2.0*near)/(right-left), 0.0, float(right+left)/(right-left), 0.0,
                     0.0, (2.0*near)/(top-bottom), float(top+bottom)/(top-bottom), 0.0,
                     0.0, 0.0, -float(far+near)/(far-near), -(2.0*far*near)/(far-near),
                     0.0, 0.0, -1.0, 0.0)
    frustum = staticmethod(frustum)
    
    def perspective(fovy, aspect, near, far):
        """ Set a perspective transformation.

        This method is equivalent to the OpenGL utility command
        gluPerspective().
        """
        top    = near * math.tan(fovy * math.pi / 360.0)
        bottom = -top
        left   = bottom * aspect
        right  = top * aspect

        return mat4.frustum(left, right, bottom, top, near, far)
    perspective = staticmethod(perspective)

    def orthographic(left, right, bottom, top, near, far):
        """Returns a matrix that represents an orthographic transformation.

        This method is equivalent to the OpenGL command glOrtho().
        """
        global _epsilon
        
        r_l = float(right-left)
        t_b = float(top-bottom)
        f_n = float(far-near)
  
        if r_l<=_epsilon:
            raise ValueError, "right-value must be greater than left-value";
        if t_b<=_epsilon:
            raise ValueError, "top-value must be greater than bottom-value"
        if f_n<=_epsilon:
            raise ValueError, "far-value must be greater than near-value"

        return mat4(2.0/r_l, 0.0, 0.0, -(right+left)/r_l,
                    0.0, 2.0/t_b, 0.0, -(top+bottom)/t_b,
                    0.0, 0.0, -2.0/f_n, -(far+near)/f_n,
                    0.0, 0.0, 0.0, 1.0)
    orthographic = staticmethod(orthographic)

    def lookAt(pos, target, up=_vec3(0,0,1)):
        """Look from pos to target.

        The resulting transformation moves the origin to pos and
        rotates so that the z-axis points to target. The y-axis is
        as close as possible to the up vector.
        """
        pos = _vec3(pos)
        target = _vec3(target)
        up = _vec3(up)
        dir = (target - pos).normalize()
        up  = up.normalize()
        up -= (up * dir) * dir
        try:
            up  = up.normalize()
        except:
            # We're looking along the up direction, so choose
            # an arbitrary direction that is perpendicular to dir
            # as new up.
            up = dir.ortho()

        right = up.cross(dir).normalize()

        return mat4(right.x, up.x, dir.x, pos.x,
                    right.y, up.y, dir.y, pos.y,
                    right.z, up.z, dir.z, pos.z,
                    0.0, 0.0, 0.0, 1.0)
    lookAt = staticmethod(lookAt)

    def ortho(self):
        """Return a matrix with orthogonal base vectors.

        Makes the x-, y- and z-axis orthogonal.
        The fourth column and row remain untouched.
        """

        m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist

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

        return mat4( x.x, y.x, z.x, m14,
                     x.y, y.y, z.y, m24,
                     x.z, y.z, z.z, m34,
                     m41, m42, m43, m44)

    def decompose(self):
        """Decomposes the matrix into a translation, rotation and scaling part.

        Returns a tuple (translation, rotation, scaling). The 
        translation and scaling parts are given as vec3's, the rotation
        is still given as a mat4.
        """
        dummy = self.ortho()
        dummy.setRow(3,_vec4(0.0, 0.0, 0.0, 1.0))
        dummy.setColumn(3,_vec4(0.0, 0.0, 0.0, 1.0))

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

        return (_vec3(self.mlist[3], self.mlist[7], self.mlist[11]),
                dummy,
                scale)

    def getMat3(self):
        """Convert to mat3 by discarding 4th row and column.
        """
        m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 = self.mlist
        return _mat3(m11,m12,m13,
                     m21,m22,m23,
                     m31,m32,m33)

    def setMat3(self, m3):
        """Sets the first three columns and rows to the values in m3.
        """
        self.mlist[0:3] = m3.mlist[0:3]
        self.mlist[4:7] = m3.mlist[3:6]
        self.mlist[8:11] = m3.mlist[6:9]

######################################################################

def _test():
    import doctest, mat4
    failed, total = doctest.testmod(mat4)
    print "%d/%d failed" % (failed, total)

if __name__=="__main__":

    _test()



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
# $Id: vec4.py,v 1.1 2005/08/15 15:39:48 mbaas Exp $

import types, math

# Comparison threshold
_epsilon = 1E-12


# vec4
class vec4(object):
    """Four-dimensional vector.

    This class represents a 4D vector.
    """

    def __init__(self, *args):
        """Constructor.

        There are several possibilities how to initialize a vector:

        v = vec4()        -> v = <0,0,0,0>
        v = vec4(a)       -> v = <a,a,a,a>
        v = vec4(x,y)     -> v = <x,y,0,0>
        v = vec4(x,y,z)   -> v = <x,y,z,0>
        v = vec4(x,y,z,w) -> v = <x,y,z,w>

        Note that specifying just one value sets all four components to
        that value.

        Additionally you can wrap those values in a list or a tuple or
        specify them as a string:

        v = vec4([1,2,3]) -> v = <1,2,3,0>
        v = vec4("4,5")   -> v = <4,5,0,0>        
        """
        
        if len(args)==0:
            self.x, self.y, self.z, self.w = (0.0, 0.0, 0.0, 0.0)

        elif len(args)==1:
            T = type(args[0])
            # scalar
            if T==types.FloatType or T==types.IntType or T==types.LongType:
                f = float(args[0])
                self.x, self.y, self.z, self.w = (f, f, f, f)
            # vec4
            elif isinstance(args[0], vec4):
                self.x, self.y, self.z, self.w = args[0]
            # Tuple/List
            elif T==types.TupleType or T==types.ListType:
                if len(args[0])==0:
                    self.x = self.y = self.z = self.w = 0.0
                elif len(args[0])==1:
                    self.x = self.y = self.z = self.w = float(args[0][0])
                elif len(args[0])==2:
                    self.x = float(args[0][0])
                    self.y = float(args[0][1])
                    self.z = 0.0
                    self.w  = 0.0
                elif len(args[0])==3:
                    x,y,z = args[0]
                    self.x = float(x)
                    self.y = float(y)
                    self.z = float(z)
                    self.w = 0.0
                elif len(args[0])==4:
                    x,y,z,w = args[0]
                    self.x = float(x)
                    self.y = float(y)
                    self.z = float(z)
                    self.w = float(w)
                else:
                    raise TypeError, "vec4() takes at most 4 arguments"
            # String
            elif T==types.StringType:
                s=args[0].replace(","," ").replace("  "," ").strip().split(" ")
                if s==[""]:
                    s=[]
                f=map(lambda x: float(x), s)
                dummy = vec4(f)
                self.x, self.y, self.z, self.w = dummy
            # error
            else:
                raise TypeError,"vec4() arg can't be converted to vec4"

        elif len(args)==2:
            self.x, self.y = (float(args[0]), float(args[1]))
            self.z, self.w = (0.0, 0.0)
            
        elif len(args)==3:
            x,y,z = args
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.w = 0.0

        elif len(args)==4:
            x,y,z,w = args
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.w = float(w)

        else:
            raise TypeError, "vec4() takes at most 4 arguments"


    def __repr__(self):
        return 'vec4('+`self.x`+', '+`self.y`+', '+`self.z`+', '+`self.w`+')'

    def __str__(self):
        fmt="%1.4f"
        return '('+fmt%self.x+', '+fmt%self.y+', '+fmt%self.z+', '+fmt%self.w+')'


    def __eq__(self, other):
        """== operator

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.6)
        >>> c=vec4(-0.3, 0.75, 0.5, 0.6)
        >>> print a==b
        0
        >>> print b==c
        1
        >>> print a==None
        0
        """
        global _epsilon
        if isinstance(other, vec4):
            return (abs(self.x-other.x)<=_epsilon and
                    abs(self.y-other.y)<=_epsilon and
                    abs(self.z-other.z)<=_epsilon and
                    abs(self.w-other.w)<=_epsilon)
        else:
            return False

    def __ne__(self, other):
        """!= operator

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.6)
        >>> c=vec4(-0.3, 0.75, 0.5, 0.6)
        >>> print a!=b
        1
        >>> print b!=c
        0
        >>> print a!=None
        1
        """
        return not (self==other)

    def __lt__(self, other):
        """< operator."""
        if isinstance(other, vec4):
            return ((self.x<other.x) and
                    (self.y<other.y) and
                    (self.z<other.z) and
                    (self.w<other.w))
        else:
            return False

    def __le__(self, other):
        """<= operator."""
        if isinstance(other, vec4):
            return ((self.x<=other.x) and
                    (self.y<=other.y) and
                    (self.z<=other.z) and
                    (self.w<=other.w))
        else:
            return False
        
    def __gt__(self, other):
        """> operator."""
        if isinstance(other, vec4):
            return ((self.x>other.x) and
                    (self.y>other.y) and
                    (self.z>other.z) and
                    (self.w>other.w))
        else:
            return False
        
    def __ge__(self, other):
        """>= operator."""
        if isinstance(other, vec4):
            return ((self.x>=other.x) and
                    (self.y>=other.y) and
                    (self.z>=other.z) and
                    (self.w>=other.w))
        else:
            return False


    def __add__(self, other):
        """Vector addition.

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.3)
        >>> print a+b
        (0.7000, 1.2500, -1.3000, 0.5000)
        """
        if isinstance(other, vec4):
            return vec4(self.x+other.x, self.y+other.y, self.z+other.z, self.w+other.w)
        else:
            raise TypeError, "unsupported operand type for +"

    def __sub__(self, other):
        """Vector subtraction.

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.3)
        >>> print a-b
        (1.3000, -0.2500, -2.3000, -0.1000)
        """
        if isinstance(other, vec4):
            return vec4(self.x-other.x, self.y-other.y, self.z-other.z, self.w-other.w)
        else:
            raise TypeError, "unsupported operand type for -"

    def __mul__(self, other):
        """Multiplication with a scalar or dot product.

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.3)
        >>> print a*2.0
        (2.0000, 1.0000, -3.6000, 0.4000)
        >>> print 2.0*a
        (2.0000, 1.0000, -3.6000, 0.4000)
        >>> print a*b
        -0.765
        """

        T = type(other)
        # vec4*scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return vec4(self.x*other, self.y*other, self.z*other, self.w*other)
        # vec4*vec4
        if isinstance(other, vec4):
            return self.x*other.x + self.y*other.y + self.z*other.z + self.w*other.w
        # unsupported
        else:
            # Try to delegate the operation to the other operand
            if getattr(other,"__rmul__",None)!=None:
                return other.__rmul__(self)
            else:
                raise TypeError, "unsupported operand type for *"

    __rmul__ = __mul__

    def __div__(self, other):
        """Division by scalar

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> print a/2.0
        (0.5000, 0.2500, -0.9000, 0.1000)
        """
        T = type(other)
        # vec4/scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return vec4(self.x/other, self.y/other, self.z/other, self.w/other)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for /"

    def __mod__(self, other):
        """Modulo (component wise)

        >>> a=vec4(3.0, 2.5, -1.8, 0.2)
        >>> print a%2.0
        (1.0000, 0.5000, 0.2000, 0.2000)
        """
        T = type(other)
        # vec4%scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            return vec4(self.x%other, self.y%other, self.z%other, self.w%other)
        # vec4%vec4
        if isinstance(other, vec4):
            return vec4(self.x%other.x, self.y%other.y, self.z%other.z, self.w%other.w)
        # unsupported
        else:
            raise TypeError, "unsupported operand type for %"

    def __iadd__(self, other):
        """Inline vector addition.

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.3)
        >>> a+=b
        >>> print a
        (0.7000, 1.2500, -1.3000, 0.5000)
        """
        if isinstance(other, vec4):
            self.x+=other.x
            self.y+=other.y
            self.z+=other.z
            self.w+=other.w
            return self
        else:
            raise TypeError, "unsupported operand type for +="

    def __isub__(self, other):
        """Inline vector subtraction.

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> b=vec4(-0.3, 0.75, 0.5, 0.3)
        >>> a-=b
        >>> print a
        (1.3000, -0.2500, -2.3000, -0.1000)
        """
        if isinstance(other, vec4):
            self.x-=other.x
            self.y-=other.y
            self.z-=other.z
            self.w-=other.w
            return self
        else:
            raise TypeError, "unsupported operand type for -="

    def __imul__(self, other):
        """Inline multiplication (only with scalar)

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> a*=2.0
        >>> print a
        (2.0000, 1.0000, -3.6000, 0.4000)
        """
        T = type(other)
        # vec4*=scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            self.x*=other
            self.y*=other
            self.z*=other
            self.w*=other
            return self
        else:
            raise TypeError, "unsupported operand type for *="

    def __idiv__(self, other):
        """Inline division with scalar

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> a/=2.0
        >>> print a
        (0.5000, 0.2500, -0.9000, 0.1000)
        """
        T = type(other)
        # vec4/=scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            self.x/=other
            self.y/=other
            self.z/=other
            self.w/=other
            return self
        else:
            raise TypeError, "unsupported operand type for /="

    def __imod__(self, other):
        """Inline modulo

        >>> a=vec4(3.0, 2.5, -1.8, 0.2)
        >>> a%=2.0
        >>> print a
        (1.0000, 0.5000, 0.2000, 0.2000)
        """
        T = type(other)
        # vec4%=scalar
        if T==types.FloatType or T==types.IntType or T==types.LongType:
            self.x%=other
            self.y%=other
            self.z%=other
            self.w%=other
            return self
        # vec4%vec4
        if isinstance(other, vec4):
            self.x%=other.x
            self.y%=other.y
            self.z%=other.z
            self.w%=other.w
            return self
        else:
            raise TypeError, "unsupported operand type for %="

    def __neg__(self):
        """Negation

        >>> a=vec4(3.0, 2.5, -1.8, 0.2)
        >>> print -a
        (-3.0000, -2.5000, 1.8000, -0.2000)
        """
        return vec4(-self.x, -self.y, -self.z, -self.w)

    def __pos__(self):
        """
        >>> a=vec4(3.0, 2.5, -1.8, 0.2)
        >>> print +a
        (3.0000, 2.5000, -1.8000, 0.2000)
        """
        return vec4(+self.x, +self.y, +self.z, +self.w)

    def __abs__(self):
        """Return the length of the vector.

        abs(v) is equivalent to v.length().

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> print abs(a)
        2.12837966538
        """
        return math.sqrt(self*self)


    def __len__(self):
        """Length of the sequence (always 4)"""
        return 4

    def __getitem__(self, key):
        """Return a component by index (0-based)

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> print a[0]
        1.0
        >>> print a[1]
        0.5
        >>> print a[2]
        -1.8
        >>> print a[3]
        0.2
        """
        T=type(key)
        if T!=types.IntType and T!=types.LongType:
            raise TypeError, "index must be integer"

        if   key==0: return self.x
        elif key==1: return self.y
        elif key==2: return self.z
        elif key==3: return self.w
        else:
            raise IndexError,"index out of range"

    def __setitem__(self, key, value):
        """Set a component by index (0-based)

        >>> a=vec4()
        >>> a[0]=1.5; a[1]=0.7; a[2]=-0.3; a[3]=0.2
        >>> print a
        (1.5000, 0.7000, -0.3000, 0.2000)
        """
        T=type(key)
        if T!=types.IntType and T!=types.LongType:
            raise TypeError, "index must be integer"

        if   key==0: self.x = value
        elif key==1: self.y = value
        elif key==2: self.z = value
        elif key==3: self.w = value
        else:
            raise IndexError,"index out of range"


    def length(self):
        """Return the length of the vector.

        v.length() is equivalent to abs(v).

        >>> a=vec4(1.0, 0.5, -1.8, 0.2)
        >>> print a.length()
        2.12837966538
        """

        return math.sqrt(self*self)

    def normalize(self):
        """Return normalized vector.

        >>> a=vec4(1.0, 0.5, -1.8, 1.2)
        >>> print a.normalize()
        (0.4107, 0.2053, -0.7392, 0.4928)
        """

        nlen = 1.0/math.sqrt(self*self)
        return vec4(self.x*nlen, self.y*nlen, self.z*nlen, self.w*nlen)

    def min(self):
        """Return the minimum value of the components.
        """
        return min(self.x, self.y, self.z, self.w)

    def max(self):
        """Return the maximum value of the components.
        """
        return max(self.x, self.y, self.z, self.w)

    def minIndex(self):
        """Return the index of the component with the minimum value.
        """
        if self.x<=self.y and self.x<=self.z and self.x<=self.w:
            return 0
        elif self.y<=self.z and self.y<=self.w:
            return 1
        elif self.z<=self.w:
            return 2
        else:
            return 3

    def maxIndex(self):
        """Return the index of the component with the maximum value.
        """
        if self.x>=self.y and self.x>=self.z and self.x>=self.w:
            return 0
        elif self.y>=self.z and self.y>=self.w:
            return 1
        elif self.z>=self.w:
            return 2
        else:
            return 3
        
    def minAbs(self):
        """Return the minimum absolute value of the components.
        """
        return min(abs(self.x), abs(self.y), abs(self.z), abs(self.w))

    def maxAbs(self):
        """Return the maximum absolute value of the components.
        """
        return max(abs(self.x), abs(self.y), abs(self.z), abs(self.w))

    def minAbsIndex(self):
        """Return the index of the component with the minimum absolute value.
        """
        ax = abs(self.x)
        ay = abs(self.y)
        az = abs(self.z)
        aw = abs(self.w)
        
        if ax<=ay and ax<=az and ax<=aw:
            return 0
        elif ay<=az and ay<=aw:
            return 1
        elif az<=aw:
            return 2
        else:
            return 3

    def maxAbsIndex(self):
        """Return the index of the component with the maximum absolute value.
        """
        ax = abs(self.x)
        ay = abs(self.y)
        az = abs(self.z)
        aw = abs(self.w)
        
        if ax>=ay and ax>=az and ax>=aw:
            return 0
        elif ay>=az and ay>=aw:
            return 1
        elif az>=aw:
            return 2
        else:
            return 3


    # "t" property (which is an alias for 'w')...
    
    def _getT(self):
        return self.w

    def _setT(self, val):
        self.w = val

    t = property(_getT, _setT, None, "4th component")



######################################################################

def _test():
    import doctest, vec4
    failed, total = doctest.testmod(vec4)
    print "%d/%d failed" % (failed, total)

if __name__=="__main__":

    _test()

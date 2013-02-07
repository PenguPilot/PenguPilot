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

# Import the modules so that the epsilon value can be changed
import vec3 as _vec3_module
import vec4 as _vec4_module
import mat3 as _mat3_module
import mat4 as _mat4_module
import quat as _quat_module

# Import types into the cgtypes namespace
from vec3 import vec3
from vec4 import vec4
from mat3 import mat3
from mat4 import mat4
from quat import quat, slerp, squad

# getEpsilon
def getEpsilon():
    """Return the epsilon threshold which is used for doing comparisons."""
    return _vec3_module._epsilon

# setEpsilon
def setEpsilon(eps):
    """Set a new epsilon threshold and returns the previously set value.
    """
    res = getEpsilon()
    eps = float(eps)
    _vec3_module._epsilon = eps
    _vec4_module._epsilon = eps
    _mat3_module._epsilon = eps
    _mat4_module._epsilon = eps
    _quat_module._epsilon = eps
    return res



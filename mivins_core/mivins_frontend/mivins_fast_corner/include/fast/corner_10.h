// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

template <class C>
inline bool Corner10(const unsigned char *p, const int w, const int barrier)
{
    const int w3 = 3 * w;
    const int t = C::prep_t(*p, barrier);
    if (!C::eval(p[-1 - w3], t))
    { // ???????????????-
        if (!C::eval(p[3 + w], t))
        { // ?????-?????????-
            return false;
        } // ?????@?????????-
        if (!C::eval(p[2 + 2 * w], t))
        { // ?????@-????????-
            return false;
        } // ?????@@????????-
        if (!C::eval(p[-1 + w3], t))
        { // ?????@@??-?????-
            return false;
        } // ?????@@??@?????-
        if (!C::eval(p[1 + w3], t))
        { // ?????@@-?@?????-
            return false;
        } // ?????@@@?@?????-
        if (!C::eval(p[w3], t))
        { // ?????@@@-@?????-
            return false;
        } // ?????@@@@@?????-
        if (!C::eval(p[-2 + 2 * w], t))
        { // ?????@@@@@-????-
            if (!C::eval(p[-w3], t))
            { // -????@@@@@-????-
                return false;
            } // @????@@@@@-????-
            if (!C::eval(p[3], t))
            { // @???-@@@@@-????-
                return false;
            } // @???@@@@@@-????-
            if (!C::eval(p[1 - w3], t))
            { // @-??@@@@@@-????-
                return false;
            } // @@??@@@@@@-????-
            if (!C::eval(p[2 - 2 * w], t))
            { // @@-?@@@@@@-????-
                return false;
            } // @@@?@@@@@@-????-
            if (!C::eval(p[3 - w], t))
            { // @@@-@@@@@@-????-
                return false;
            } // @@@@@@@@@@-????-
            return true;
        } // ?????@@@@@@????-
        if (!C::eval(p[-3 + w], t))
        { // ?????@@@@@@-???-
            if (!C::eval(p[3], t))
            { // ????-@@@@@@-???-
                return false;
            } // ????@@@@@@@-???-
            if (!C::eval(p[1 - w3], t))
            { // ?-??@@@@@@@-???-
                return false;
            } // ?@??@@@@@@@-???-
            if (!C::eval(p[2 - 2 * w], t))
            { // ?@-?@@@@@@@-???-
                return false;
            } // ?@@?@@@@@@@-???-
            if (!C::eval(p[3 - w], t))
            { // ?@@-@@@@@@@-???-
                return false;
            } // ?@@@@@@@@@@-???-
            return true;
        } // ?????@@@@@@@???-
        if (!C::eval(p[3], t))
        { // ????-@@@@@@@???-
            if (!C::eval(p[-3], t))
            { // ????-@@@@@@@-??-
                return false;
            } // ????-@@@@@@@@??-
            if (!C::eval(p[-3 - w], t))
            { // ????-@@@@@@@@-?-
                return false;
            } // ????-@@@@@@@@@?-
            if (!C::eval(p[-2 - 2 * w], t))
            { // ????-@@@@@@@@@--
                return false;
            } // ????-@@@@@@@@@@-
            return true;
        } // ????@@@@@@@@???-
        if (!C::eval(p[3 - w], t))
        { // ???-@@@@@@@@???-
            if (!C::eval(p[-3], t))
            { // ???-@@@@@@@@-??-
                return false;
            } // ???-@@@@@@@@@??-
            if (!C::eval(p[-3 - w], t))
            { // ???-@@@@@@@@@-?-
                return false;
            } // ???-@@@@@@@@@@?-
            return true;
        } // ???@@@@@@@@@???-
        if (!C::eval(p[-3], t))
        { // ???@@@@@@@@@-??-
            if (!C::eval(p[2 - 2 * w], t))
            { // ??-@@@@@@@@@-??-
                return false;
            } // ??@@@@@@@@@@-??-
            return true;
        } // ???@@@@@@@@@@??-
        return true;
    } // ???????????????@
    if (!C::eval(p[-2 - 2 * w], t))
    { // ??????????????-@
        if (!C::eval(p[3], t))
        { // ????-?????????-@
            return false;
        } // ????@?????????-@
        if (!C::eval(p[3 + w], t))
        { // ????@-????????-@
            return false;
        } // ????@@????????-@
        if (!C::eval(p[w3], t))
        { // ????@@??-?????-@
            return false;
        } // ????@@??@?????-@
        if (!C::eval(p[1 + w3], t))
        { // ????@@?-@?????-@
            return false;
        } // ????@@?@@?????-@
        if (!C::eval(p[2 + 2 * w], t))
        { // ????@@-@@?????-@
            return false;
        } // ????@@@@@?????-@
        if (!C::eval(p[3 - w], t))
        { // ???-@@@@@?????-@
            if (!C::eval(p[-1 + w3], t))
            { // ???-@@@@@-????-@
                return false;
            } // ???-@@@@@@????-@
            if (!C::eval(p[-3 - w], t))
            { // ???-@@@@@@???--@
                return false;
            } // ???-@@@@@@???@-@
            if (!C::eval(p[-2 + 2 * w], t))
            { // ???-@@@@@@-??@-@
                return false;
            } // ???-@@@@@@@??@-@
            if (!C::eval(p[-3 + w], t))
            { // ???-@@@@@@@-?@-@
                return false;
            } // ???-@@@@@@@@?@-@
            if (!C::eval(p[-3], t))
            { // ???-@@@@@@@@-@-@
                return false;
            } // ???-@@@@@@@@@@-@
            return true;
        } // ???@@@@@@?????-@
        if (!C::eval(p[2 - 2 * w], t))
        { // ??-@@@@@@?????-@
            if (!C::eval(p[-3], t))
            { // ??-@@@@@@???-?-@
                return false;
            } // ??-@@@@@@???@?-@
            if (!C::eval(p[-1 + w3], t))
            { // ??-@@@@@@-??@?-@
                return false;
            } // ??-@@@@@@@??@?-@
            if (!C::eval(p[-2 + 2 * w], t))
            { // ??-@@@@@@@-?@?-@
                return false;
            } // ??-@@@@@@@@?@?-@
            if (!C::eval(p[-3 + w], t))
            { // ??-@@@@@@@@-@?-@
                return false;
            } // ??-@@@@@@@@@@?-@
            return true;
        } // ??@@@@@@@?????-@
        if (!C::eval(p[1 - w3], t))
        { // ?-@@@@@@@?????-@
            if (!C::eval(p[-1 + w3], t))
            { // ?-@@@@@@@-????-@
                return false;
            } // ?-@@@@@@@@????-@
            if (!C::eval(p[-2 + 2 * w], t))
            { // ?-@@@@@@@@-???-@
                return false;
            } // ?-@@@@@@@@@???-@
            if (!C::eval(p[-3 + w], t))
            { // ?-@@@@@@@@@-??-@
                return false;
            } // ?-@@@@@@@@@@??-@
            return true;
        } // ?@@@@@@@@?????-@
        if (!C::eval(p[-w3], t))
        { // -@@@@@@@@?????-@
            if (!C::eval(p[-1 + w3], t))
            { // -@@@@@@@@-????-@
                return false;
            } // -@@@@@@@@@????-@
            if (!C::eval(p[-2 + 2 * w], t))
            { // -@@@@@@@@@-???-@
                return false;
            } // -@@@@@@@@@@???-@
            return true;
        } // @@@@@@@@@?????-@
        return true;
    } // ??????????????@@
    if (!C::eval(p[-3 - w], t))
    { // ?????????????-@@
        if (!C::eval(p[1 + w3], t))
        { // ???????-?????-@@
            return false;
        } // ???????@?????-@@
        if (!C::eval(p[3 - w], t))
        { // ???-???@?????-@@
            return false;
        } // ???@???@?????-@@
        if (!C::eval(p[3], t))
        { // ???@-??@?????-@@
            return false;
        } // ???@@??@?????-@@
        if (!C::eval(p[3 + w], t))
        { // ???@@-?@?????-@@
            return false;
        } // ???@@@?@?????-@@
        if (!C::eval(p[2 + 2 * w], t))
        { // ???@@@-@?????-@@
            return false;
        } // ???@@@@@?????-@@
        if (!C::eval(p[2 - 2 * w], t))
        { // ??-@@@@@?????-@@
            if (!C::eval(p[w3], t))
            { // ??-@@@@@-????-@@
                return false;
            } // ??-@@@@@@????-@@
            if (!C::eval(p[-3], t))
            { // ??-@@@@@@???--@@
                return false;
            } // ??-@@@@@@???@-@@
            if (!C::eval(p[-1 + w3], t))
            { // ??-@@@@@@-??@-@@
                return false;
            } // ??-@@@@@@@??@-@@
            if (!C::eval(p[-2 + 2 * w], t))
            { // ??-@@@@@@@-?@-@@
                return false;
            } // ??-@@@@@@@@?@-@@
            if (!C::eval(p[-3 + w], t))
            { // ??-@@@@@@@@-@-@@
                return false;
            } // ??-@@@@@@@@@@-@@
            return true;
        } // ??@@@@@@?????-@@
        if (!C::eval(p[1 - w3], t))
        { // ?-@@@@@@?????-@@
            if (!C::eval(p[-3 + w], t))
            { // ?-@@@@@@???-?-@@
                return false;
            } // ?-@@@@@@???@?-@@
            if (!C::eval(p[w3], t))
            { // ?-@@@@@@-??@?-@@
                return false;
            } // ?-@@@@@@@??@?-@@
            if (!C::eval(p[-1 + w3], t))
            { // ?-@@@@@@@-?@?-@@
                return false;
            } // ?-@@@@@@@@?@?-@@
            if (!C::eval(p[-2 + 2 * w], t))
            { // ?-@@@@@@@@-@?-@@
                return false;
            } // ?-@@@@@@@@@@?-@@
            return true;
        } // ?@@@@@@@?????-@@
        if (!C::eval(p[-w3], t))
        { // -@@@@@@@?????-@@
            if (!C::eval(p[w3], t))
            { // -@@@@@@@-????-@@
                return false;
            } // -@@@@@@@@????-@@
            if (!C::eval(p[-1 + w3], t))
            { // -@@@@@@@@-???-@@
                return false;
            } // -@@@@@@@@@???-@@
            if (!C::eval(p[-2 + 2 * w], t))
            { // -@@@@@@@@@-??-@@
                return false;
            } // -@@@@@@@@@@??-@@
            return true;
        } // @@@@@@@@?????-@@
        return true;
    } // ?????????????@@@
    if (!C::eval(p[-w3], t))
    { // -????????????@@@
        if (!C::eval(p[2 + 2 * w], t))
        { // -?????-??????@@@
            return false;
        } // -?????@??????@@@
        if (!C::eval(p[1 + w3], t))
        { // -?????@-?????@@@
            return false;
        } // -?????@@?????@@@
        if (!C::eval(p[-2 + 2 * w], t))
        { // -?????@@??-??@@@
            return false;
        } // -?????@@??@??@@@
        if (!C::eval(p[w3], t))
        { // -?????@@-?@??@@@
            return false;
        } // -?????@@@?@??@@@
        if (!C::eval(p[-1 + w3], t))
        { // -?????@@@-@??@@@
            return false;
        } // -?????@@@@@??@@@
        if (!C::eval(p[-3 + w], t))
        { // -?????@@@@@-?@@@
            if (!C::eval(p[1 - w3], t))
            { // --????@@@@@-?@@@
                return false;
            } // -@????@@@@@-?@@@
            if (!C::eval(p[3 + w], t))
            { // -@???-@@@@@-?@@@
                return false;
            } // -@???@@@@@@-?@@@
            if (!C::eval(p[2 - 2 * w], t))
            { // -@-??@@@@@@-?@@@
                return false;
            } // -@@??@@@@@@-?@@@
            if (!C::eval(p[3 - w], t))
            { // -@@-?@@@@@@-?@@@
                return false;
            } // -@@@?@@@@@@-?@@@
            if (!C::eval(p[3], t))
            { // -@@@-@@@@@@-?@@@
                return false;
            } // -@@@@@@@@@@-?@@@
            return true;
        } // -?????@@@@@@?@@@
        if (!C::eval(p[-3], t))
        { // -?????@@@@@@-@@@
            if (!C::eval(p[3 + w], t))
            { // -????-@@@@@@-@@@
                return false;
            } // -????@@@@@@@-@@@
            if (!C::eval(p[2 - 2 * w], t))
            { // -?-??@@@@@@@-@@@
                return false;
            } // -?@??@@@@@@@-@@@
            if (!C::eval(p[3 - w], t))
            { // -?@-?@@@@@@@-@@@
                return false;
            } // -?@@?@@@@@@@-@@@
            if (!C::eval(p[3], t))
            { // -?@@-@@@@@@@-@@@
                return false;
            } // -?@@@@@@@@@@-@@@
            return true;
        } // -?????@@@@@@@@@@
        return true;
    } // @????????????@@@
    if (!C::eval(p[-3], t))
    { // @???????????-@@@
        if (!C::eval(p[2 + 2 * w], t))
        { // @?????-?????-@@@
            return false;
        } // @?????@?????-@@@
        if (!C::eval(p[2 - 2 * w], t))
        { // @?-???@?????-@@@
            return false;
        } // @?@???@?????-@@@
        if (!C::eval(p[3 - w], t))
        { // @?@-??@?????-@@@
            return false;
        } // @?@@??@?????-@@@
        if (!C::eval(p[3 + w], t))
        { // @?@@?-@?????-@@@
            return false;
        } // @?@@?@@?????-@@@
        if (!C::eval(p[3], t))
        { // @?@@-@@?????-@@@
            return false;
        } // @?@@@@@?????-@@@
        if (!C::eval(p[1 - w3], t))
        { // @-@@@@@?????-@@@
            if (!C::eval(p[1 + w3], t))
            { // @-@@@@@-????-@@@
                return false;
            } // @-@@@@@@????-@@@
            if (!C::eval(p[-3 + w], t))
            { // @-@@@@@@???--@@@
                return false;
            } // @-@@@@@@???@-@@@
            if (!C::eval(p[w3], t))
            { // @-@@@@@@-??@-@@@
                return false;
            } // @-@@@@@@@??@-@@@
            if (!C::eval(p[-1 + w3], t))
            { // @-@@@@@@@-?@-@@@
                return false;
            } // @-@@@@@@@@?@-@@@
            if (!C::eval(p[-2 + 2 * w], t))
            { // @-@@@@@@@@-@-@@@
                return false;
            } // @-@@@@@@@@@@-@@@
            return true;
        } // @@@@@@@?????-@@@
        return true;
    } // @???????????@@@@
    if (!C::eval(p[1 - w3], t))
    { // @-??????????@@@@
        if (!C::eval(p[1 + w3], t))
        { // @-?????-????@@@@
            return false;
        } // @-?????@????@@@@
        if (!C::eval(p[-3 + w], t))
        { // @-?????@???-@@@@
            return false;
        } // @-?????@???@@@@@
        if (!C::eval(p[w3], t))
        { // @-?????@-??@@@@@
            return false;
        } // @-?????@@??@@@@@
        if (!C::eval(p[-1 + w3], t))
        { // @-?????@@-?@@@@@
            return false;
        } // @-?????@@@?@@@@@
        if (!C::eval(p[-2 + 2 * w], t))
        { // @-?????@@@-@@@@@
            return false;
        } // @-?????@@@@@@@@@
        return true;
    } // @@??????????@@@@
    if (!C::eval(p[2 - 2 * w], t))
    { // @@-?????????@@@@
        if (!C::eval(p[-3 + w], t))
        { // @@-????????-@@@@
            return false;
        } // @@-????????@@@@@
        if (!C::eval(p[w3], t))
        { // @@-?????-??@@@@@
            return false;
        } // @@-?????@??@@@@@
        if (!C::eval(p[-1 + w3], t))
        { // @@-?????@-?@@@@@
            return false;
        } // @@-?????@@?@@@@@
        if (!C::eval(p[-2 + 2 * w], t))
        { // @@-?????@@-@@@@@
            return false;
        } // @@-?????@@@@@@@@
        return true;
    } // @@@?????????@@@@
    if (!C::eval(p[-3 + w], t))
    { // @@@????????-@@@@
        if (!C::eval(p[3 - w], t))
        { // @@@-???????-@@@@
            return false;
        } // @@@@???????-@@@@
        if (!C::eval(p[3], t))
        { // @@@@-??????-@@@@
            return false;
        } // @@@@@??????-@@@@
        if (!C::eval(p[3 + w], t))
        { // @@@@@-?????-@@@@
            return false;
        } // @@@@@@?????-@@@@
        return true;
    } // @@@????????@@@@@
    if (!C::eval(p[-2 + 2 * w], t))
    { // @@@???????-@@@@@
        if (!C::eval(p[3 - w], t))
        { // @@@-??????-@@@@@
            return false;
        } // @@@@??????-@@@@@
        if (!C::eval(p[3], t))
        { // @@@@-?????-@@@@@
            return false;
        } // @@@@@?????-@@@@@
        return true;
    } // @@@???????@@@@@@
    if (!C::eval(p[3 - w], t))
    { // @@@-??????@@@@@@
        if (!C::eval(p[-1 + w3], t))
        { // @@@-?????-@@@@@@
            return false;
        } // @@@-?????@@@@@@@
        return true;
    } // @@@@??????@@@@@@
    return true;
}

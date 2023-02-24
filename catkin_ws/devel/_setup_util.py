#!/nix/store/yd3nnf1rd6xjxmz2dyllf0kw06hd2nkw-ros-env/bin/python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""This file generates shell code for the setup.SHELL scripts to set environment variables."""

from __future__ import print_function

import argparse
import copy
import errno
import os
import platform
import sys

CATKIN_MARKER_FILE = '.catkin'

system = platform.system()
IS_DARWIN = (system == 'Darwin')
IS_WINDOWS = (system == 'Windows')

PATH_TO_ADD_SUFFIX = ['bin']
if IS_WINDOWS:
    # while catkin recommends putting dll's into bin, 3rd party packages often put dll's into lib
    # since Windows finds dll's via the PATH variable, prepend it with path to lib
    PATH_TO_ADD_SUFFIX.extend(['lib'])

# subfolder of workspace prepended to CMAKE_PREFIX_PATH
ENV_VAR_SUBFOLDERS = {
    'CMAKE_PREFIX_PATH': '',
    'LD_LIBRARY_PATH' if not IS_DARWIN else 'DYLD_LIBRARY_PATH': 'lib',
    'PATH': PATH_TO_ADD_SUFFIX,
    'PKG_CONFIG_PATH': os.path.join('lib', 'pkgconfig'),
    'PYTHONPATH': 'lib/python3.10/site-packages',
}


def rollback_env_variables(environ, env_var_subfolders):
    """
    Generate shell code to reset environment variables.

    by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH.
    This does not cover modifications performed by environment hooks.
    """
    lines = []
    unmodified_environ = copy.copy(environ)
    for key in sorted(env_var_subfolders.keys()):
        subfolders = env_var_subfolders[key]
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        value = _rollback_env_variable(unmodified_environ, key, subfolders)
        if value is not None:
            environ[key] = value
            lines.append(assignment(key, value))
    if lines:
        lines.insert(0, comment('reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH'))
    return lines


def _rollback_env_variable(environ, name, subfolders):
    """
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolders: list of str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    """
    value = environ[name] if name in environ else ''
    env_paths = [path for path in value.split(os.pathsep) if path]
    value_modified = False
    for subfolder in subfolders:
        if subfolder:
            if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
                subfolder = subfolder[1:]
            if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
                subfolder = subfolder[:-1]
        for ws_path in _get_workspaces(environ, include_fuerte=True, include_non_existing=True):
            path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
            path_to_remove = None
            for env_path in env_paths:
                env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
                if env_path_clean == path_to_find:
                    path_to_remove = env_path
                    break
            if path_to_remove:
                env_paths.remove(path_to_remove)
                value_modified = True
    new_value = os.pathsep.join(env_paths)
    return new_value if value_modified else None


def _get_workspaces(environ, include_fuerte=False, include_non_existing=False):
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces.

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    value = environ[env_name] if env_name in environ else ''
    paths = [path for path in value.split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte')) or (include_non_existing and not os.path.exists(path))]
    return workspaces


def prepend_env_variables(environ, env_var_subfolders, workspaces):
    """Generate shell code to prepend environment variables for the all workspaces."""
    lines = []
    lines.append(comment('prepend folders of workspaces to environment variables'))

    paths = [path for path in workspaces.split(os.pathsep) if path]

    prefix = _prefix_env_variable(environ, 'CMAKE_PREFIX_PATH', paths, '')
    lines.append(prepend(environ, 'CMAKE_PREFIX_PATH', prefix))

    for key in sorted(key for key in env_var_subfolders.keys() if key != 'CMAKE_PREFIX_PATH'):
        subfolder = env_var_subfolders[key]
        prefix = _prefix_env_variable(environ, key, paths, subfolder)
        lines.append(prepend(environ, key, prefix))
    return lines


def _prefix_env_variable(environ, name, paths, subfolders):
    """
    Return the prefix to prepend to the environment variable NAME.

    Adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    """
    value = environ[name] if name in environ else ''
    environ_paths = [path for path in value.split(os.pathsep) if path]
    checked_paths = []
    for path in paths:
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        for subfolder in subfolders:
            path_tmp = path
            if subfolder:
                path_tmp = os.path.join(path_tmp, subfolder)
            # skip nonexistent paths
            if not os.path.exists(path_tmp):
                continue
            # exclude any path already in env and any path we already added
            if path_tmp not in environ_paths and path_tmp not in checked_paths:
                checked_paths.append(path_tmp)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def assignment(key, value):
    if not IS_WINDOWS:
        return 'export %s="%s"' % (key, value)
    else:
        return 'set %s=%s' % (key, value)


def comment(msg):
    if not IS_WINDOWS:
        return '# %s' % msg
    else:
        return 'REM %s' % msg


def prepend(environ, key, prefix):
    if key not in environ or not environ[key]:
        return assignment(key, prefix)
    if not IS_WINDOWS:
        return 'export %s="%s$%s"' % (key, prefix, key)
    else:
        return 'set %s=%s%%%s%%' % (key, prefix, key)


def find_env_hooks(environ, cmake_prefix_path):
    """Generate shell code with found environment hooks for the all workspaces."""
    lines = []
    lines.append(comment('found environment hooks in workspaces'))

    generic_env_hooks = []
    generic_env_hooks_workspace = []
    specific_env_hooks = []
    specific_env_hooks_workspace = []
    generic_env_hooks_by_filename = {}
    specific_env_hooks_by_filename = {}
    generic_env_hook_ext = 'bat' if IS_WINDOWS else 'sh'
    specific_env_hook_ext = environ['CATKIN_SHELL'] if not IS_WINDOWS and 'CATKIN_SHELL' in environ and environ['CATKIN_SHELL'] else None
    # remove non-workspace paths
    workspaces = [path for path in cmake_prefix_path.split(os.pathsep) if path and os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    for workspace in reversed(workspaces):
        env_hook_dir = os.path.join(workspace, 'etc', 'catkin', 'profile.d')
        if os.path.isdir(env_hook_dir):
            for filename in sorted(os.listdir(env_hook_dir)):
                if filename.endswith('.%s' % generic_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in generic_env_hooks_by_filename:
                        i = generic_env_hooks.index(generic_env_hooks_by_filename[filename])
                        generic_env_hooks.pop(i)
                        generic_env_hooks_workspace.pop(i)
                    # append env hook
                    generic_env_hooks.append(os.path.join(env_hook_dir, filename))
                    generic_env_hooks_workspace.append(workspace)
                    generic_env_hooks_by_filename[filename] = generic_env_hooks[-1]
                elif specific_env_hook_ext is not None and filename.endswith('.%s' % specific_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in specific_env_hooks_by_filename:
                        i = specific_env_hooks.index(specific_env_hooks_by_filename[filename])
                        specific_env_hooks.pop(i)
                        specific_env_hooks_workspace.pop(i)
                    # append env hook
                    specific_env_hooks.append(os.path.join(env_hook_dir, filename))
                    specific_env_hooks_workspace.append(workspace)
                    specific_env_hooks_by_filename[filename] = specific_env_hooks[-1]
    env_hooks = generic_env_hooks + specific_env_hooks
    env_hooks_workspace = generic_env_hooks_workspace + specific_env_hooks_workspace
    count = len(env_hooks)
    lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_COUNT', count))
    for i in range(count):
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d' % i, env_hooks[i]))
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d_WORKSPACE' % i, env_hooks_workspace[i]))
    return lines


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    parser.add_argument('--extend', action='store_true', help='Skip unsetting previous environment variables to extend context')
    parser.add_argument('--local', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    return parser.parse_known_args(args=args)[0]


if __name__ == '__main__':
    try:
        try:
            args = _parse_arguments()
        except Exception as e:
            print(e, file=sys.stderr)
            sys.exit(1)

        if not args.local:
            # environment at generation time
            CMAKE_PREFIX_PATH = r'/nix/store/yd3nnf1rd6xjxmz2dyllf0kw06hd2nkw-ros-env;/home/konstantin/labs/catkin_ws/devel;/nix/store/f14n28cmvwiifqx9hx8h1f5famkjf9lf-ignition-cmake2-2.14.0;/nix/store/5cip8xpjb9g10nwyzyslxpck3anc4r4h-pkg-config-wrapper-0.29.2;/nix/store/pz1dsi25virxsk7r3070k396wdq25qk3-ignition-cmake-0.6.1;/nix/store/mhq6vmzb6w1q5kicv91x9ywqb57fb2x2-patchelf-0.15.0;/nix/store/iiq295j1z4q1sxmdbrl2j8ma3l5ns4wr-gcc-wrapper-11.3.0;/nix/store/7xs3mva2z3z4hf48mh6d5alcin52qbm9-binutils-wrapper-2.39;/nix/store/8kl2vsyflx859lz3n8lc3z6v1dgc36cs-glibc-locales-2.35-224;/nix/store/4acvgdlm8d0na492skvc40l49bdajnas-gazebo-11.11.0;/nix/store/p7ga27hhi2f3m2nsqa9zrsgyzgjy515x-freeimage-unstable-2021-11-01;/nix/store/yyq2xy779h3i450wimpqp36hb2wz6wwi-boost-1.79.0-dev;/nix/store/vd7kn0ghz4q2ca0dn8p828g32gbg46bh-boost-1.79.0;/nix/store/g57a0hbcvx6ph0a144s0fq0n0j2pqww6-protobuf-3.21.12;/nix/store/i5gpi18f7xq27ajsi9kjdmhc6s1m6vnr-sdformat-9.9.0;/nix/store/9c02mb56sc8yv9fndj53vs1r8mhj9vyc-ignition-math6-6.12.0;/nix/store/g561glln7902ps9pcdhr924vq5idd27g-tinyxml-2.6.2;/nix/store/hcbalan0xq30hb53myvw93i309ifh4yg-tbb-2020.3;/nix/store/frkq7wdq6v62wwxj0dfj3dsj7bxbs273-ogre-1.9.1;/nix/store/g5ngy54lssjlhl58wlqrchdmc8hngkxr-ffmpeg-4.4.3-dev;/nix/store/lhdvsw44gcsrd8sb3yrynw9mv5qbn6w1-ffmpeg-4.4.3-bin;/nix/store/nimzw0j39l4vdzd3d3hhz33vply6i3dw-ffmpeg-4.4.3;/nix/store/zrmwrfw64wn6b4dhb2dx0kbhbl3jjnm2-ignition-math4-4.0.0;/nix/store/9vpxniqx32myl5nzmg99yvpfg84hv9h0-ignition-transport8-8.3.0;/nix/store/q5g0699im7kkibb9z0l5fv426azxg6kr-cppzmq-4.9.0;/nix/store/52gssbvmlrbx3vka6j64chihl2a3lmn4-zeromq-4.3.4;/nix/store/f9cbd0k09das2am31hg4p3vh8xq852y5-util-linux-minimal-2.38.1-dev;/nix/store/4ba3is25i2zs52wwbgjljkhbs413wazy-util-linux-minimal-2.38.1-bin;/nix/store/a2d7gak8xa3ks4qj090gvqm4wvb51rb8-util-linux-minimal-2.38.1-lib;/nix/store/6h701cg4hbipzzbm8rcis7iyfn2r5f0d-ignition-msgs5-5.10.0;/nix/store/np44prj0bf6515ihwi3lnii7bgi0qx89-tinyxml-2-6.0.0;/nix/store/2xl10h06j2sh549n8ba68mms9vp1hdch-ignition-fuel-tools4-4.5.0;/nix/store/9936dhr2kgpdpa30y4bbrn3h88az5mqk-ignition-common3-3.14.1;/nix/store/arrxja9h6jxh9qs5iw3vxcznlkq0w00i-curl-7.87.0-dev;/nix/store/jrshnwn6jsrdyywx7l0bza365s98232j-brotli-1.0.9-dev;/nix/store/xkz1bbszzvlz832rjsyzs24ir91rppiz-brotli-1.0.9-lib;/nix/store/ym33yw6zxhxg6gljxmznq3k9ipjzpsg9-brotli-1.0.9;/nix/store/sb5csxly72wqy2nlhml53mg64p0yghg4-libkrb5-1.20.1-dev;/nix/store/5x3yh3mn6lqhjshbn7l5rjj8kcavk0qw-libkrb5-1.20.1;/nix/store/4zgy23fpmaw53s47ahvarwh68jbw8zbh-nghttp2-1.51.0-dev;/nix/store/zvpsk7zakxpkc554pa5fdbz74dswgyyh-nghttp2-1.51.0-bin;/nix/store/bwdrs69xi81q1b445j1jm0ncwyxabi21-nghttp2-1.51.0-lib;/nix/store/57qbywmcd1wilxm5s2qf9ba322p08ps0-libidn2-2.3.2-dev;/nix/store/5z0cnyw08jkcjm5gz7jmbrirlch1x8h0-libidn2-2.3.2-bin;/nix/store/s99my0m4dcxi75m03m4qdr0hvlip3590-libidn2-2.3.2;/nix/store/rrqpfz324x8835wvn1x99d84j8mzv2wd-openssl-3.0.7-dev;/nix/store/bx2qlfli3b8crrxcw7ab0lgv02c886yf-openssl-3.0.7-bin;/nix/store/773pradjpvxgxgs0mcklisx8ly6k1r4f-openssl-3.0.7;/nix/store/kzqlxp1dkpgqgcq22bqykdr9kl4s0cbh-libssh2-1.10.0-dev;/nix/store/nf1xgi7gyxww3bbw83svpankvl3y5hbh-libssh2-1.10.0;/nix/store/2v7ip5fqwhp5r00xhvhgixi80wm932zn-zlib-1.2.13-dev;/nix/store/04c0b1rmi9r6k9sl69a4gw3mhp3b5q2n-zlib-1.2.13;/nix/store/m78q7x70xln86cga0iw086j794cf6dfy-zstd-1.5.2-dev;/nix/store/kh6xf15jsy31qgm3l17nd6ngpfml0lq0-zstd-1.5.2-bin;/nix/store/q22rqwl4z3dl06nb8rrki7j6zmpq0040-zstd-1.5.2;/nix/store/22jzbbwy648x7r9hil77j478fkq9jggs-curl-7.87.0-bin;/nix/store/24aw9ykmz7hgkwvwf3fq2bv2ilivsm8c-curl-7.87.0;/nix/store/hmz0zchr87w3gz4v8ygxjq1s58h30dqi-jsoncpp-1.9.4-dev;/nix/store/k0zbgnm3w85lfjh7p2s993whw70990im-jsoncpp-1.9.4;/nix/store/dikh5c5871q79jvbk53l4y725bb7h400-libyaml-0.2.5;/nix/store/a452hizbpg5r9mxlmzw7mr15w8mcp3n2-libzip-1.9.2-dev;/nix/store/z4rfmsc5m1qmjxpzx88c0j0bhfrivdh4-libzip-1.9.2;/nix/store/j0mq22m4pkizxxpi98x1bbjd3nf7mzfs-cmake-3.24.3;/nix/store/wk1rmhf5ikxs5364fw8h7ib86c7hkcj2-catkin-setup-hook;/nix/store/wsgfapgg7alvc30xvq0hcds23hsl9vf1-hook;/nix/store/kihqrjm7762ys4xwbljm66s3v3mzvfi1-hook;/nix/store/saj2hfgxzl35zy7x8nz5qyy4n2hga5sb-hook;/nix/store/2mhap4va3y0m2nr1kihc1vnw242hllh4-gtest-1.12.1-dev;/nix/store/wsv18l0yqh4pifbc9mdri611x8xy4r18-gtest-1.12.1;/nix/store/7mqlh78ch05q48g10jmgkgn8may9vxym-boost-1.79.0-dev;/nix/store/ycjal93qlg6wxrrx5diyz7z0fri1jllx-boost-1.79.0;/nix/store/abax98471z8fshv4b9p46bkh3lxmpy0z-python3-3.10.9;/nix/store/kgcxc7rl142hyk1ypakiv1h7s2sn66mr-urdfdom-3.1.0;/nix/store/hsm4dg1qncxn5g0gniwcjj9jv3hdzyya-urdfdom-headers-1.1.0;/nix/store/c0xx40xc3mfxwh2h5gjp5jqk84k28x8z-console-bridge-1.0.2;/nix/store/6dzma1gsq59q57v0h1kk3cq6gsm6p17f-graphviz-7.0.2;/nix/store/89bfzx1a8j293z7gd6283hi0dy8n1y1x-opencv-4.7.0;/nix/store/ybr660054mpq1nl4yglric96fqwhf8xs-util-linux-2.38.1-dev;/nix/store/g1fhps9i336pn0mxbllvd3x06ivah2ia-util-linux-2.38.1-bin;/nix/store/3vbw25ldfxpr5syj55yzvcggy4dcaxr0-util-linux-2.38.1-lib;/nix/store/kjpf61f0ccnq642bsl2gyih2x5dlv6xw-apr-1.7.0-dev;/nix/store/8qz55c61s0lhq5a7axd2fvhs8gx9r535-apr-1.7.0;/nix/store/r54wxadx89q7zxv6dk58ffndm08n12b1-log4cxx-0.10.0;/nix/store/dm8vzadkp4xcxi4ibmwx9710bifih7x8-libyaml-cpp-0.7.0;/nix/store/31gnxmmbfdlzzvv3dp9vjfzbjq76kyzh-poco-1.11.1-dev;/nix/store/yybcrgxf8815xq7iknlsz21fl88a0j0y-pcre-8.45-dev;/nix/store/iz5iz7rba0iyxqk9h46fl2kf3i5qlgyf-pcre-8.45-bin;/nix/store/lx2yaqgvxhcwj65w65q539c2rgxh2bb7-pcre-8.45;/nix/store/hf4c5sf2w21ab0kwavbn0z9z14zip866-expat-2.5.0-dev;/nix/store/q5n1f03cdjj76a5kcpldajkjczdjl3jy-expat-2.5.0;/nix/store/w7828xfm8r7xj5jik8qv1kbz80imqcyf-sqlite-3.40.1-dev;/nix/store/dc9s7rw9bv809gb6fs51a9mbdhl14gmb-sqlite-3.40.1-bin;/nix/store/8izb9ahnpnb93wqrv24ip897v82qw80b-sqlite-3.40.1;/nix/store/yz1i5mg3icrpkzrsyrh2p7wc1b82m6vn-poco-1.11.1;/nix/store/nb7aqh5wclz2fr9rs8gj40vjnk4ns04w-bzip2-1.0.8-dev;/nix/store/phwcwazlcabg1v6wwrdv6gr5mh2r3ld8-bzip2-1.0.8-bin;/nix/store/86gmg0dan51jsm3p2n2fnac1s8mq94i7-bzip2-1.0.8;/nix/store/6wdy9p3rmp035bkg1mb1vksyarvd9dwp-gpgme-1.18.0-dev;/nix/store/7gp6sgjxwr4bzvy4r29a0rmb476pxww0-glib-2.74.3-dev;/nix/store/s7dns7drgskk7xpr4s3kvqcc745dg9js-libffi-3.4.4-dev;/nix/store/fynz7l60dnqphimnm6l090pckv5scx7b-libffi-3.4.4;/nix/store/n162rpxjvh57qgmpdj90sg4b9f59k0sr-gettext-0.21;/nix/store/17ahh52bwnky55vxss7nnq35mmbmyfmw-glibc-iconv-2.35;/nix/store/2nik04ccmv6cfvsl28mfk32mnv797vr2-glib-2.74.3-bin;/nix/store/i4dqcpppyyq5yqcvw95mv5s11yfyy8pf-glib-2.74.3;/nix/store/caqrap5vap25ygpkfr1wwd0c4gn3269s-libassuan-2.5.5-dev;/nix/store/6cjvf8ch1r4780zahmni0i58wr9cyp1b-libassuan-2.5.5;/nix/store/2444s28c4v19czjd2djm4yw16knnngds-libgpg-error-1.45-dev;/nix/store/74z70avsylmd2z5nx4g2hr3n5x69hglv-libgpg-error-1.45;/nix/store/n9h1778rfl1xzmwkd0c74wk6zh3z2kgw-pth-2.0.7;/nix/store/1n1n83qm5i13rxx0hwxmjk51ypaazzdk-gpgme-1.18.0;/nix/store/7k01a1jmsmqiraksv48frqdvrxfb1cwn-lz4-1.9.4-dev;/nix/store/s1d0083bxyba438gwqbzc0ibff1c529p-lz4-1.9.4-bin;/nix/store/8x1wc41zxsw3szqfp932knmgn571h9ws-lz4-1.9.4'.split(';')
        else:
            # don't consider any other prefix path than this one
            CMAKE_PREFIX_PATH = []
        # prepend current workspace if not already part of CPP
        base_path = os.path.dirname(__file__)
        # CMAKE_PREFIX_PATH uses forward slash on all platforms, but __file__ is platform dependent
        # base_path on Windows contains backward slashes, need to be converted to forward slashes before comparison
        if os.path.sep != '/':
            base_path = base_path.replace(os.path.sep, '/')

        if base_path not in CMAKE_PREFIX_PATH:
            CMAKE_PREFIX_PATH.insert(0, base_path)
        CMAKE_PREFIX_PATH = os.pathsep.join(CMAKE_PREFIX_PATH)

        environ = dict(os.environ)
        lines = []
        if not args.extend:
            lines += rollback_env_variables(environ, ENV_VAR_SUBFOLDERS)
        lines += prepend_env_variables(environ, ENV_VAR_SUBFOLDERS, CMAKE_PREFIX_PATH)
        lines += find_env_hooks(environ, CMAKE_PREFIX_PATH)
        print('\n'.join(lines))

        # need to explicitly flush the output
        sys.stdout.flush()
    except IOError as e:
        # and catch potential "broken pipe" if stdout is not writable
        # which can happen when piping the output to a file but the disk is full
        if e.errno == errno.EPIPE:
            print(e, file=sys.stderr)
            sys.exit(2)
        raise

    sys.exit(0)

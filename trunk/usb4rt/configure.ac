#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)

AC_INIT([usb4rt],[0.0.2],[],[])

CONFIG_USB4RT_VERSION_MAJOR=0
CONFIG_USB4RT_VERSION_MINOR=0
CONFIG_USB4RT_REVISION_LEVEL=2

AC_CONFIG_HEADERS([config/usb4rt_config_pre.h])
AC_CONFIG_AUX_DIR([config/autoconf])
AC_CONFIG_MACRO_DIR([config/m4])
AC_CONFIG_SRCDIR([core/rt_usb_core.c],
                 [host/rt_uhci.c],
                 [prog/rt_usb_ping.c
                ])
AC_PREFIX_DEFAULT([/usr/local/usb4rt])
AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_PROG_INSTALL

CFLAGS=${CFLAGS:--s}

# Checks for programs.
AC_ARG_WITH(CC,
    AS_HELP_STRING([--with-cc=compiler], [use specific C compiler]),
    [case "$withval" in
    "" | y | ye | yes | n | no)
  AC_MSG_ERROR([*** You must supply an argument to --with-cc.]) ;;
    esac
    CC="$withval"])
AC_PROG_CC

AC_DEFINE_UNQUOTED(CONFIG_USB4RT_BUILD_STRING, "$build", [Build system alias])
USB4RT_BUILD_STRING="$build"
AC_DEFINE_UNQUOTED(CONFIG_USB4RT_HOST_STRING, "$host", [Host system alias])
USB4RT_HOST_STRING="$host"
AC_DEFINE_UNQUOTED(CONFIG_USB4RT_COMPILER, "`$CC -v 2>&1 | tail -n 1`" , [Compiler])

AM_INIT_AUTOMAKE([foreign no-exeext dist-bzip2])
AM_MAINTAINER_MODE
AC_PROG_LIBTOOL
AM_PROG_AS
AC_SUBST(LD)

##########################
# Checks for libraries.
##########################

##########################
# Checks for header files.
##########################

##########################
# Checks for typedefs, structures, and compiler characteristics.
##########################
AC_C_INLINE

##########################
# Checks for library functions.
##########################
AC_HEADER_STDC
AC_CHECK_FUNCS([memset])

#
# checking arch
#
case "$host" in
  i*86*-*)
    USB4RT_TARGET_ARCH=i386
    wanted_kernel_arch=CONFIG_X86
    ;;
 *) echo ""
   echo "*******************************************"
   echo "Still unsupported host: $host -- Sorry."
   echo "*******************************************"
   echo ""
   exit 1;;
esac

#
# Load the USB4RT configuration.
#

USB4RT_KCONFIG_FILE=$srcdir/config/defconfig
kconfig_type=default

AC_ARG_WITH(kconfig-file,
    AS_HELP_STRING([--with-kconfig-file=<kconfig-file>], [KConfig parameter file]),
    [
  case "$withval" in
  "" | y | ye | yes | n | no)
      AC_MSG_ERROR([You must supply an argument to --with-kconfig-file.])
    ;;
  esac
  USB4RT_KCONFIG_FILE="$withval"
  kconfig_type=supplied
    ])

AC_MSG_CHECKING([for USB4RT Kconfig file])
if test \! -f $USB4RT_KCONFIG_FILE; then
    AC_MSG_ERROR([Kconfig-file $USB4RT_KCONFIG_FILE not found -- --with-kconfig-file sets Kconfig file])
fi
. $USB4RT_KCONFIG_FILE
AC_MSG_RESULT([$USB4RT_KCONFIG_FILE ($kconfig_type)])

#
# ARG-Enable Common-Debug (default=enabled)
#
AC_MSG_CHECKING(for debug-messages [common])
AC_ARG_ENABLE(dbg_common,
  AS_HELP_STRING([--disable-dbg-common], [Disable common debug messages (default: enabled)]),
  [case "$enableval" in
  n | no) unset CONFIG_USB4RT_DBG_COMMON ;;
  *) CONFIG_USB4RT_DBG_COMMON=y ;;
  esac])
AC_MSG_RESULT(${CONFIG_USB4RT_DBG_COMMON:-yes})
test x$CONFIG_USB4RT_DBG_COMMON = xy && AC_DEFINE(CONFIG_USB4RT_DBG_COMMON,1,[Kconfig])
AM_CONDITIONAL(CONFIG_USB4RT_DBG_COMMON,[test x$CONFIG_USB4RT_DBG_COMMON = xy])

#
# ARG-Enable Time-Debug (default=disabled)
#
AC_MSG_CHECKING(for debug-messages [timing])
AC_ARG_ENABLE(dbg_time,
  AS_HELP_STRING([--enable-dbg-time], [Enable time debug messages (default: disabled)]),
  [case "$enableval" in
  y | yes) CONFIG_USB4RT_DBG_TIME=y ;;
  *) unset CONFIG_USB4RT_DBG_TIME ;;
  esac])
AC_MSG_RESULT(${CONFIG_USB4RT_DBG_TIME:-no})
test x$CONFIG_USB4RT_DBG_TIME = xy && AC_DEFINE(CONFIG_USB4RT_DBG_TIME,1,[Kconfig])
AM_CONDITIONAL(CONFIG_USB4RT_DBG_TIME,[test x$CONFIG_USB4RT_DBG_TIME = xy])

#
# ARG-Enable Transfer-Descriptor-Debug (default=disabled)
#
AC_MSG_CHECKING(for debug-messages [transfer-descriptor])
AC_ARG_ENABLE(dbg_td,
  AS_HELP_STRING([--enable-dbg-td], [Enable transfer-descriptor debug messages (default: disabled)]),
  [case "$enableval" in
  y | yes) CONFIG_USB4RT_DBG_TD=y ;;
  *) unset CONFIG_USB4RT_DBG_TD ;;
  esac])
AC_MSG_RESULT(${CONFIG_USB4RT_DBG_TD:-no})
test x$CONFIG_USB4RT_DBG_TD = xy && AC_DEFINE(CONFIG_USB4RT_DBG_TD,1,[Kconfig])
AM_CONDITIONAL(CONFIG_USB4RT_DBG_TD,[test x$CONFIG_USB4RT_DBG_TD = xy])

#
# ARG-Enable Queue-Head-Debug (default=disabled)
#
AC_MSG_CHECKING(for debug-messages [queue-heads])
AC_ARG_ENABLE(dbg_qh,
  AS_HELP_STRING([--enable-dbg-qh], [Enable queue-head debug messages (default: disabled)]),
  [case "$enableval" in
  y | yes) CONFIG_USB4RT_DBG_QH=y ;;
  *) unset CONFIG_USB4RT_DBG_QH ;;
  esac])
AC_MSG_RESULT(${CONFIG_USB4RT_DBG_QH:-no})
test x$CONFIG_USB4RT_DBG_QH = xy && AC_DEFINE(CONFIG_USB4RT_DBG_QH,1,[Kconfig])
AM_CONDITIONAL(CONFIG_USB4RT_DBG_QH,[test x$CONFIG_USB4RT_DBG_QH = xy])

#
# ARG-Enable Bandwidth-Reclamation (default=enabled)
#
AC_MSG_CHECKING(for debug-messages [timing])
AC_ARG_ENABLE(bandw_reclam,
  AS_HELP_STRING([--disable-bandw-reclam], [Disable bandwith-reclamation for Fullspeed Bulk- and Control-Transfer (default: enabled)]),
  [case "$enableval" in
  n | no) unset CONFIG_USB4RT_BANDW_RECLAM ;;
  *) CONFIG_USB4RT_BANDW_RECLAM=y ;;
  esac])
AC_MSG_RESULT(${CONFIG_USB4RT_BANDW_RECLAM:-yes})
test x$CONFIG_USB4RT_BANDW_RECLAM = xy && AC_DEFINE(CONFIG_USB4RT_BANDW_RECLAM,1,[Kconfig])
AM_CONDITIONAL(CONFIG_USB4RT_BANDW_RECLAM,[test x$CONFIG_USB4RT_BANDW_RECLAM = xy])



#
# ARG-With rtai-fusion-path
#
AC_ARG_WITH(rtai,
  AS_HELP_STRING([--with-rtai=RTAI], [path to installation of fusion ]),
    [case "${withval}" in
      "" | y | ye | yes | n | no)
        AC_MSG_ERROR([You must supply an argument to --with-rtai.]) ;;
    esac],
    with_rtai=[${CONFIG_USB4RT_FUSIONDIR}])

#
# try to find rtai-fusion
# the presence of rtai-config is a good indication for rtai >= 3
#
AC_MSG_CHECKING([for RTAI fusion installation])
RTAI_CONFIG="${with_rtai}/bin/rtai-config"
if test -x "${RTAI_CONFIG}"; then
  RTAI_DIR="`cd ${with_rtai} && pwd`" # let the shell normalize the path
  AC_MSG_RESULT([${RTAI_DIR}])
else
  AC_MSG_RESULT([no])
fi

#
# check if rtai-fusion is installed with DESTDIR
# e.g.: 'make DESTDIR=/foo install'
#
AC_MSG_CHECKING([for RTAI DESTDIR])
USB4RT_RTAI_PREFIX="`cd \`${RTAI_CONFIG} --prefix\` && pwd`"     # let the shell normalize the path

if test "${USB4RT_RTAI_PREFIX}" \!= "${RTAI_DIR}"; then
  USB4RT_RTAI_DESTDIR="`cd \`echo ${RTAI_DIR} | sed -e s,${USB4RT_RTAI_PREFIX}$,,\` && pwd`"
  export DESTDIR="${USB4RT_RTAI_DESTDIR}"

  USB4RT_RTAI_PREFIX="`cd \`${RTAI_CONFIG} --prefix\` && pwd`"   # let the shell normalize the path
  if test "${USB4RT_RTAI_PREFIX}" = "${RTAI_DIR}"; then
    AC_MSG_RESULT([${DESTDIR}])
  else
    AC_MSG_ERROR([*** Cannot identify DESTDIR])
  fi
else
  AC_MSG_RESULT([none])
fi

#
#++++ RTAI-Version ++++++++++++++++
#

AC_MSG_CHECKING([for RTAI-fusion version])
CONFIG_USB4RT_RTAI_PRE_VERSION="`${RTAI_CONFIG} --version`"

case "$CONFIG_USB4RT_RTAI_PRE_VERSION" in
  0.8.3-fusion | 0.8.99-fusion | 0.9*-fusion)
    CONFIG_USB4RT_RTAI_VERSION=fusion-083
    AC_DEFINE(CONFIG_FUSION_083, 1, [We build for fusion 0.8.3 or better])
    ;;
  *)
    AC_MSG_ERROR([*** Unsupported RTAI-fusion version $CONFIG_USB4RT_RTAI_PRE_VERSION in $RTAI_DIR])
  ;;
esac
AC_MSG_RESULT([${CONFIG_USB4RT_RTAI_VERSION}])

AM_CONDITIONAL(CONFIG_FUSION,[test "$CONFIG_USB4RT_RTAI_VERSION" = fusion-083])

#
#++++ RTAI-config-file ++++++++++++++++
#

AC_MSG_CHECKING([for RTAI-fusion config file])
CONFIG_USB4RT_RTAI_CONFIG_FILE="`${RTAI_CONFIG} --config`"
if test \! -f $CONFIG_USB4RT_RTAI_CONFIG_FILE; then
  AC_MSG_ERROR([*** RTAI-fusion configuration file not found])
fi
AC_MSG_RESULT([${CONFIG_USB4RT_RTAI_CONFIG_FILE}])

#
# source (load) the rtai-fusion configuration
#
. ${CONFIG_USB4RT_RTAI_CONFIG_FILE}

#
#++++ RTAI-config ++++++++++++++++
#
case "$CONFIG_USB4RT_RTAI_VERSION" in
  fusion-083)

    # check for native RTAI skin, needed by USB4RT
    AC_MSG_CHECKING([for RTAI native skin])
    case "${CONFIG_RTAI_SKIN_NATIVE}" in
      y) AC_MSG_RESULT([${CONFIG_RTAI_SKIN_NATIVE}]) ;;
      *) AC_MSG_ERROR([*** Please enable native RTAI skin]) ;;
    esac

    # check for semas, needed by USB4RT
    AC_MSG_CHECKING([for RTAI Semaphores])
    case "${CONFIG_RTAI_OPT_NATIVE_SEM}" in
      y) AC_MSG_RESULT([${CONFIG_RTAI_OPT_NATIVE_SEM}]) ;;
      *) AC_MSG_ERROR([*** Please enable semaphores in RTAI]) ;;
    esac
      ;;

    *) # ---> oops <----
    AC_MSG_ERROR([*** internal error])
      ;;
    esac

#
# import settings from RTAI
#
case "${CONFIG_USB4RT_RTAI_VERSION}" in
  fusion-083)
    RTAI_USER_CFLAGS="`${RTAI_CONFIG} --fusion-cflags`"
    RTAI_KMOD_CFLAGS="`${RTAI_CONFIG} --fusion-cflags`"
    CONFIG_USB4RT_RTAI_ARCH="`${RTAI_CONFIG} --arch`"
    RTAI_LIBRARIES="`${RTAI_CONFIG} --library-dir`"
    ;;
  *)
    AC_MSG_ERROR([*** internal error])
    ;;
esac

#
# architecture tests....
#
AC_MSG_CHECKING([for RTAI target arch])
AC_MSG_RESULT([${CONFIG_USB4RT_RTAI_ARCH}])

AC_MSG_CHECKING([for USB4RT target arch])
AC_MSG_RESULT([${USB4RT_TARGET_ARCH}])

if test x"${USB4RT_TARGET_ARCH}" != x"${CONFIG_USB4RT_RTAI_ARCH}"; then
    AC_MSG_ERROR([*** RTAI and USB4RT target architectures are not the same])
fi

case "${CONFIG_USB4RT_RTAI_ARCH}" in
  i386)
    AC_DEFINE(CONFIG_ARCH_I386, 1,[we are building for x86])
  ;;
  *)
    AC_MSG_ERROR([*** unsupported architecture])
    ;;
esac
AC_SUBST(USB4RT_TARGET_ARCH)

#
# get linux kernel tree and version
#
RTAI_LINUX_DIR="`${RTAI_CONFIG} --linux-dir`"
RTAI_LINUX_VERSION="`${RTAI_CONFIG} --linux-version`"
AC_SUBST(RTAI_LINUX_DIR)

#
# check linux kernel makefile and .config
#
AC_MSG_CHECKING([for Linux source tree])
if test \! -r ${RTAI_LINUX_DIR}/Makefile; then
   echo
   AC_MSG_ERROR([*** No Linux kernel tree in ${RTAI_LINUX_DIR}])
fi

if test \! -r ${RTAI_LINUX_DIR}/.config; then
   echo
   AC_MSG_ERROR([*** $RTAI_LINUX_DIR is not configured])
fi

#
# source (load) the kernel configuration
#
. ${RTAI_LINUX_DIR}/.config
AC_MSG_RESULT([$RTAI_LINUX_DIR (kernel ${RTAI_LINUX_VERSION}${CONFIG_SMP:+-SMP})])

#
# some checks
#
if test x${CONFIG_MODULES} = x; then
  AC_MSG_ERROR([*** Please set CONFIG_MODULES for $RTAI_LINUX_DIR and rebuild your kernel])
fi

if test x${CONFIG_MODVERSIONS} = xy; then
  AC_MSG_ERROR([*** Please unset CONFIG_MODVERSIONS for $RTAI_LINUX_DIR and rebuild your kernel])
fi

if test x${CONFIG_RTAI_ADEOS} = xy -a x${CONFIG_ADEOS_CORE} = x; then
   AC_MSG_ERROR([*** Please activate the Adeos support (General menu) in $RTAI_LINUX_DIR and rebuild your kernel])
fi

#
# crosscompiling
#
AC_MSG_CHECKING([for CROSS_COMPILE])
if test "${CROSS_COMPILE}" = ""; then
  CROSS_COMPILE="`echo ${CC} | sed s/gcc//`"

  if test "${CROSS_COMPILE}gcc" \!= "${CC}"; then
    CROSS_COMPILE=""
    AC_MSG_RESULT([Cannot identify CROSS_COMPILE - assuming none - set manually if needed])
  else
    if test "${CROSS_COMPILE}" = ""; then
      AC_MSG_RESULT([none])
    else
      AC_MSG_RESULT([${CROSS_COMPILE}])
    fi
  fi
else
  AC_MSG_RESULT([${CROSS_COMPILE}])
fi
AC_SUBST(CROSS_COMPILE)

#
# check linux version and set specific parameters
#
unset CONFIG_KBUILD
case "${RTAI_LINUX_VERSION}" in
  2.6.*)
    USB4RT_MODULE_EXT=.ko
    CONFIG_KBUILD=y
    ;;
  *)
    AC_MSG_ERROR([*** Unsupported kernel version $RTAI_LINUX_VERSION])
    ;;
esac
AC_SUBST(USB4RT_MODULE_EXT)
AM_CONDITIONAL(CONFIG_KBUILD,[test "$CONFIG_KBUILD" = y])

USB4RT_KBUILD_ENV='src2obj = $(patsubst %.$(1),%.o,$(filter %.$(1),$(call notdir,$(2))))'
USB4RT_KBUILD_CMD=" \
  @for src in \$(filter-out FORCE, \$^); do            \
    if test \\! -r \`basename \$\$src\`; then          \
      \$(LN_S) \$\$src;                                \
    fi;                                                \
  done;                                                \
  \$(MAKE) -C $RTAI_LINUX_DIR                          \
  CROSS_COMPILE=\$(CROSS_COMPILE)                      \
    ARCH=\$(USB4RT_TARGET_ARCH)                         \
    M=\"\`pwd\`\"                                      \
    V=\$(V)                                            \
    rtai_dir=\"${USB4RT_RTAI_PREFIX}\"                  \
    top_srcdir=\"\`cd \$(top_srcdir) && pwd\`\"        \
    top_builddir=\"\`cd \$(top_builddir) && pwd\`\"    \
    srcdir=\"\`cd \$(srcdir) && pwd\`\"                \
    build_objs='\$(call src2obj,c,\$^)'                \
    build_target='\$(basename \$@)'                    \
    modules                                            \
    "
USB4RT_KBUILD_CLEAN="                                   \
  @rm -f *.ko *.mod.c .*.cmd *.o;                      \
  for src in \$^; do                                   \
    if test -h \`basename \$\$src\`; then              \
      rm -f \`basename \$\$src\`;                      \
    fi;                                                \
  done;                                                \
  if test -d .tmp_versions; then                       \
    rm -rf .tmp_versions;                              \
  fi;                                                  \
  "

#
# Hack to suppress undefined symbol warnings during MODPOST stage ===
#
if test -e $RTAI_LINUX_DIR/Module.symvers; then
  mv $RTAI_LINUX_DIR/Module.symvers $RTAI_LINUX_DIR/Module.symvers.let_modpost_be_quiet
fi
if test -e $RTAI_LINUX_DIR/vmlinux; then
  mv $RTAI_LINUX_DIR/vmlinux $RTAI_LINUX_DIR/vmlinux.let_modpost_be_quiet
fi


AC_MSG_CHECKING([for kernel module extension])
AC_MSG_RESULT([$USB4RT_MODULE_EXT])

AC_SUBST(USB4RT_MODULE_EXT)
AC_SUBST(USB4RT_KBUILD_ENV)
AC_SUBST(USB4RT_KBUILD_CMD)
AC_SUBST(USB4RT_KBUILD_CLEAN)

bs_kmodext=$USB4RT_MODULE_EXT
bs_kcompile="make -C $RTAI_LINUX_DIR ARCH=$USB4RT_TARGET_ARCH CROSS_COMPILE=$CROSS_COMPILE V=1 M=`pwd` SUBDIRS=`pwd` modules"

#
# RTAI header checks
#

ac_save_CPPFLAGS="${CPPFLAGS}"
CPPFLAGS="${ac_save_CPPFLAGS} ${RTAI_KMOD_CFLAGS}"

# Working around some Autoconf ugliness here: names of variables
# stored into the cache are canonicalized, turning /'s into _'s. So if
# you need to match X_Y.h and X/Y.h as different filenames, you are
# toast. Whichever comes first will be cached as "X_Y_h" and the
# second one will never get checked in the fs, but simply
# uncached. This applies to rtai_sem.h and rtai/sem.h for instance.
# To prevent this, we must only check a single set of non-conflicting
# headers, depending on the RTAI version which has been
# identified. (rpm)
case "${CONFIG_USB4RT_RTAI_VERSION}" in
  fusion-083)
    BS_CHECK_KHEADERS([rtai/sem.h rtai/mutex.h rtai/pipe.h],
      [],
      [AC_MSG_ERROR([*** header not found or working, please check RTAI installation])],
      [
        #include <linux/kernel.h>
      ])
    ;;
  *)
    AC_MSG_ERROR([*** internal error])
    ;;
esac

CPPFLAGS="${ac_save_CFLAGS}"

#
# Installation directory for modules
#
AC_MSG_CHECKING([for module installation directory])

AC_ARG_WITH(module-dir,
    AS_HELP_STRING([--with-module-dir=<module-dir>], [Installation directory for modules]),
    [case "${withval}" in
      "" | y | ye | yes | n | no)
        AC_MSG_ERROR([You must supply an argument to --with-module-dir.]) ;;
    esac],
    with_module_dir='${exec_prefix}/modules')

USB4RT_MODULE_DIR="$with_module_dir"

AC_MSG_RESULT([$USB4RT_MODULE_DIR])

AC_SUBST(USB4RT_MODULE_DIR)

#
# create links to Makefiles used by linux-2.6.x
#
for DIR in \
  core \
  host \
  prog \
  ; do
      AC_CONFIG_LINKS(${DIR}/Makefile:${DIR}/Makefile.kbuild)
done

dnl
dnl Build a Kconfig file corresponding to the current configuration
dnl if none exists.
dnl

if test \! -r .rtai_config; then
   set | grep -e '^CONFIG_USB4RT' -e '^CONFIG_MODULES' | \
   sed -e 's,\(.*\)=\(.*\)$,\1=\"\2\",' > .usb4rt_config
fi

##########################
# Output files.
##########################
AC_CONFIG_FILES([GNUmakefile \
                 core/GNUmakefile \
                 host/GNUmakefile \
                 prog/GNUmakefile
                ])
AC_OUTPUT

#
# generate include/usb4rt_config.h
#
AC_MSG_NOTICE([creating core/usb4rt_config.h])
sed -e "s/ PACKAGE/ USB4RT_PACKAGE/" -e "s/ VERSION/ USB4RT_VERSION/" \
    config/usb4rt_config_pre.h > core/usb4rt_config.h.new
if cmp -s core/usb4rt_config.h.new core/usb4rt_config.h; then
    rm core/usb4rt_config.h.new
    AC_MSG_NOTICE([core/usb4rt_config.h is unchanged])
else
    mv core/usb4rt_config.h.new core/usb4rt_config.h
fi

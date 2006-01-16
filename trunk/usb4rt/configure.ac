#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)

AC_INIT([usb4rt],m4_normalize(m4_include([config/version])),usb4rt-users@lists.berlios.de)

CONFIG_USB4RT_VERSION_MAJOR=`expr $PACKAGE_VERSION : '\([0-9]*\)'`
CONFIG_USB4RT_VERSION_MINOR=`expr $PACKAGE_VERSION : '[0-9]*\.\([0-9]*\)'`
CONFIG_USB4RT_REVISION_LEVEL=`expr $PACKAGE_VERSION : '[0-9]*\.[0-9]*\.\([0-9]*\)'`
test "x$CONFIG_USB4RT_REVISION_LEVEL" = "x" && CONFIG_USB4RT_REVISION_LEVEL=0

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

dnl ======================================================================
dnl checking cc
dnl ======================================================================

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

AC_C_INLINE
AC_HEADER_STDC
AC_CHECK_FUNCS([memset])

dnl ============================================================================
dnl  checking target arch
dnl ============================================================================

AC_MSG_CHECKING([for USB4RT target architecture])
case "$host" in
  i*86*-*)
    USB4RT_TARGET_ARCH=i386
    wanted_kernel_arch=CONFIG_X86
    ;;
 *)
    AC_MSG_ERROR([ *** Sorry, unsupported architecture])
    ;;
esac
AC_MSG_RESULT([${USB4RT_TARGET_ARCH}])

dnl ============================================================================
dnl  Load the USB4RT configuration.
dnl ============================================================================

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

dnl ======================================================================
dnl  LINUX kernel with XENOMAI
dnl ======================================================================

AC_ARG_WITH(linux-dir,
    AS_HELP_STRING([--with-linux-dir=<Linux-Sources>], [Linux source directory]),
    [case "${withval}" in
        "" | y | ye | yes | n | no)
            AC_MSG_ERROR([You must supply an argument to --with-linux-dir.])
      ;;
    esac
    CONFIG_USB4RT_LINUXDIR="$withval"
    ])

dnl -----------------------------------------------------------------
dnl check config file
dnl -----------------------------------------------------------------

AC_MSG_CHECKING([for Linux config file])
if test \! -f ${CONFIG_USB4RT_LINUXDIR}/.config ; then
  AC_MSG_ERROR([*** Please configure your kernel])
fi
AC_MSG_RESULT([${CONFIG_USB4RT_LINUXDIR}/.config])

dnl -----------------------------------------------------------------
dnl get linux version
dnl -----------------------------------------------------------------

AC_MSG_CHECKING([for Linux version])
LINUX_VERSION="` cat ${CONFIG_USB4RT_LINUXDIR}/.config | \
                 grep "# Linux kernel version:" | \
                 gawk -F ": " '{print $2}'`"
AC_MSG_RESULT([${LINUX_VERSION}])

dnl -----------------------------------------------------------------
dnl check version
dnl -----------------------------------------------------------------

case "${LINUX_VERSION}" in
    2.6.*)
        ;;
    *)
        AC_MSG_ERROR([*** Unsupported kernel version $LINUX_VERSION])
        ;;
esac

dnl -----------------------------------------------------------------
dnl loading linux config
dnl -----------------------------------------------------------------

. ${CONFIG_USB4RT_LINUXDIR}/.config

dnl -----------------------------------------------------------------
dnl  checking needed Xenomai and skins in linux config
dnl -----------------------------------------------------------------

AC_MSG_CHECKING([for Xenomai])
    case "${CONFIG_XENOMAI}" in
        y) AC_MSG_RESULT([${CONFIG_XENOMAI}]) ;;
        *) AC_MSG_ERROR([*** Please enable Xenomai]) ;;
    esac

AC_MSG_CHECKING([for Xenomai native skin])
    case "${CONFIG_XENO_SKIN_NATIVE}" in
        y | m) AC_MSG_RESULT([${CONFIG_XENO_SKIN_NATIVE}]) ;;
        *) AC_MSG_ERROR([*** Please enable native skin]) ;;
    esac

dnl ======================================================================
dnl  XENOMAI userspace
dnl ======================================================================

AC_ARG_WITH(xeno-user-dir,
    AS_HELP_STRING([--with-xeno-user-dir=<Xenomai>], [Xenomai userspace installation path]),
    [case "${withval}" in
        "" | y | ye | yes | n | no)
            AC_MSG_ERROR([You must supply an argument to --with-xeno-user-dir.])
      ;;
    esac
    CONFIG_USB4RT_XENOMAIDIR="$withval"
    ])

dnl -----------------------------------------------------------------
dnl  try to find Xenomai user
dnl -----------------------------------------------------------------

AC_MSG_CHECKING([for Xenomai userspace installation])
XENO_USER_CONFIG="${CONFIG_USB4RT_XENOMAIDIR}/bin/xeno-config"
if test -x "${XENO_USER_CONFIG}"; then
  XENO_USER_DIR="`cd ${CONFIG_USB4RT_XENOMAIDIR} && pwd`" # let the shell normalize the path
  AC_MSG_RESULT([${XENO_USER_DIR}])
else
  AC_MSG_ERROR([no Xenomai found in $CONFIG_USB4RT_XENOMAIDIR])
fi

dnl -----------------------------------------------------------------
dnl  get Xenomai user prefix
dnl -----------------------------------------------------------------

AC_MSG_CHECKING([for Xenomai userspace PREFIX])
XENO_USER_PREFIX="`cd \`${XENO_USER_CONFIG} --prefix\` && pwd`"     # let the shell normalize the path
AC_MSG_RESULT([${XENO_USER_PREFIX}])

AC_MSG_CHECKING([for Xenomai userspace DESTDIR])
if test "${XENO_USER_PREFIX}" \!= "${XENO_USER_DIR}"; then
  XENO_DESTDIR="`cd \`echo ${XENO_USER_DIR} | sed -e s,${XENO_USER_PREFIX}$,,\` && pwd`"
  export DESTDIR="${XENO_DESTDIR}"

  XENO_USER_PREFIX="`cd \`${XENO_USER_CONFIG} --prefix\` && pwd`"   # let the shell normalize the path
  if test "${XENO_USER_PREFIX}" = "${XENO_USER_DIR}"; then
    AC_MSG_RESULT([${DESTDIR}])
  else
    AC_MSG_ERROR([*** Cannot identify DESTDIR])
  fi
else
  AC_MSG_RESULT([none])
fi

dnl -----------------------------------------------------------------
dnl  checking Xenomai version
dnl -----------------------------------------------------------------

AC_MSG_CHECKING([for Xenomai version])
XENO_VERSION_PRE="`${XENO_USER_CONFIG} --version`"

case "$XENO_VERSION_PRE" in
  2.1* | 2.0.9* )
    XENO_VERSION=xeno-21x
    ;;
  *)
    AC_MSG_ERROR([*** Unsupported Xenomai version $XENO_VERSION_PRE in $XENO_USER_DIR])
  ;;
esac
AC_MSG_RESULT([${XENO_VERSION_PRE}])

dnl -----------------------------------------------------------------
dnl  get some more information about Xenomai
dnl -----------------------------------------------------------------

XENO_USER_CFLAGS="`${XENO_USER_CONFIG} --xeno-cflags`"
XENO_USER_ARCH="`${XENO_USER_CONFIG} --arch`"
XENO_LIBRARIES="`${XENO_USER_CONFIG} --library-dir`"

dnl -----------------------------------------------------------------
dnl export Xenomai values
dnl -----------------------------------------------------------------

AC_SUBST(XENO_USER_PREFIX)
AC_SUBST(XENO_LIBRARIES)
AC_DEFINE(CONFIG_XENO_21X, 1, [We build for Xenomai 2.1 or newer])

dnl ======================================================================
dnl  architecture test Xenomai <-> usb4rt
dnl ======================================================================

AC_MSG_CHECKING([for Xenomai target arch])
AC_MSG_RESULT([${XENO_USER_ARCH}])

AC_MSG_CHECKING([for USB4RT target arch])
AC_MSG_RESULT([${USB4RT_TARGET_ARCH}])

if test x"${USB4RT_TARGET_ARCH}" != x"${XENO_USER_ARCH}"; then
    AC_MSG_ERROR([*** Xenomai and USB4RT target architectures are not the same])
fi

case "${XENO_USER_ARCH}" in
    i386)
        AC_DEFINE(CONFIG_ARCH_I386, 1,[we are building for x86])
        ;;
    *)
        AC_MSG_ERROR([*** unsupported architecture])
        ;;
esac

AC_SUBST(USB4RT_TARGET_ARCH)

dnl ======================================================================
dnl             crosscompiling
dnl ======================================================================

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

dnl ============================================================================
dnl  build for 2.6 kernel
dnl ============================================================================

USB4RT_MODULE_EXT=.ko
USB4RT_KBUILD_ENV='src2obj = $(patsubst %.$(1),%.o,$(filter %.$(1),$(call notdir,$(2))))'
USB4RT_KBUILD_CMD=" \
  @for src in \$(filter-out FORCE, \$^); do               \
    if test \\! -r \`basename \$\$src\`; then             \
      \$(LN_S) \$\$src;                                   \
    fi;                                                   \
  done;                                                   \
  \$(MAKE) -C $CONFIG_USB4RT_LINUXDIR                     \
    CROSS_COMPILE=\$(CROSS_COMPILE)                       \
    ARCH=\$(USB4RT_TARGET_ARCH)                           \
    M=\"\`pwd\`\"                                         \
    V=\$(V)                                               \
    linux_dir=\"${CONFIG_USB4RT_LINUXDIR}\"               \
    xeno_user_dir=\"${XENO_USER_PREFIX}\"                 \
    top_srcdir=\"\`cd \$(top_srcdir) && pwd\`\"           \
    top_builddir=\"\`cd \$(top_builddir) && pwd\`\"       \
    srcdir=\"\`cd \$(srcdir) && pwd\`\"                   \
    build_objs='\$(call src2obj,c,\$^)'                   \
    build_target='\$(basename \$@)'                       \
    MODVERDIR=\"\`cd \$(top_builddir) && pwd\`/symbols\"  \
    modules                                               \
    "
USB4RT_KBUILD_CLEAN="                                     \
  @rm -f *.ko *.mod.c .*.cmd *.o;                         \
  for src in \$^; do                                      \
    if test -h \`basename \$\$src\`; then                 \
      rm -f \`basename \$\$src\`;                         \
    fi;                                                   \
  done;                                                   \
  if test -d .tmp_versions; then                          \
    rm -rf .tmp_versions;                                 \
  fi;                                                     \
  "

AC_MSG_CHECKING([for kernel module extension])
AC_MSG_RESULT([$USB4RT_MODULE_EXT])

AC_SUBST(USB4RT_MODULE_EXT)
AC_SUBST(USB4RT_KBUILD_ENV)
AC_SUBST(USB4RT_KBUILD_CMD)
AC_SUBST(USB4RT_KBUILD_CLEAN)

bs_kmodext=$USB4RT_MODULE_EXT
bs_kcompile="make -C $CONFIG_USB4RT_LINUXDIR ARCH=$USB4RT_TARGET_ARCH
             CROSS_COMPILE=$CROSS_COMPILE V=1 M=`pwd` SUBDIRS=`pwd` modules"

dnl ======================================================================
dnl             Installation directory for modules
dnl ======================================================================

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

dnl ======================================================================
dnl             hac^H^H^Hfix problem with multible defined symbols problem
dnl             shamelessly stolen from RTAI-3.1 tnx to Philippe
dnl ======================================================================

dnl CAUTION: We need to have the CONFIG_USB4RT_XX symbols always defined when
dnl the configuration header is read, but we want the Autoconf-produced
dnl symbols to be defined only when compiling RTNET. This way, we won't
dnl pollute the namespace with the latter when our configuration header is
dnl indirectly included by a client application. To achieve this, we ask
dnl autoheader to produce the following header structure:
dnl #define CONFIG_XX
dnl #define CONFIG_XX ...
dnl #ifdef __IN_RTNET__
dnl <Autoconf-defined symbols>
dnl #endif /* __IN_USB4RT__ */
dnl This is quite a hack since we have to rely on the fact that
dnl all Autoconf-generated symbols are lexicographically sorted
dnl after CONFIG_USB4RT_XX ones, but, well...

dnl Use a key which will cause the verbatim string to be put after
dnl all CONFIG_USB4RT_XX symbols, but still before any Autoconf-generated
dnl symbol, hence CONFIG_USB4RT___.
AH_VERBATIM(CONFIG_USB4RT___,[#ifdef __IN_USB4RT__])

dnl Now we can close the conditional section, right after all
dnl Autoconf-generated symbols have been listed.
AH_BOTTOM([#endif /* __IN_USB4RT__ */])

XENO_KMOD_CFLAGS="-D__IN_USB4RT__ $XENO_KMOD_CFLAGS"
XENO_USER_CFLAGS="-D__IN_USB4RT__ $XENO_USER_CFLAGS"

AC_SUBST(XENO_KMOD_CFLAGS)
AC_SUBST(XENO_USER_CFLAGS)

dnl ======================================================================
dnl                      components
dnl ======================================================================

dnl -----------------------------------------------------------------
dnl  ARG-Enable Common-Debug (default=enabled)
dnl -----------------------------------------------------------------

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

dnl -----------------------------------------------------------------
dnl  ARG-Enable Time-Debug (default=disabled)
dnl -----------------------------------------------------------------

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

dnl -----------------------------------------------------------------
dnl  ARG-Enable Transfer-Descriptor-Debug (default=disabled)
dnl -----------------------------------------------------------------

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

dnl -----------------------------------------------------------------
dnl  ARG-Enable Queue-Head-Debug (default=disabled)
dnl -----------------------------------------------------------------

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

dnl -----------------------------------------------------------------
dnl  ARG-Enable Bandwidth-Reclamation (default=enabled)
dnl -----------------------------------------------------------------

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

dnl -----------------------------------------------------------------
dnl  ...
dnl -----------------------------------------------------------------


dnl ======================================================================
dnl             create links to Makefiles used by linux-2.6.x
dnl ======================================================================

for DIR in \
  core \
  host \
  prog \
  ; do
      AC_CONFIG_LINKS(${DIR}/Makefile:${DIR}/Makefile.kbuild)
done

dnl ======================================================================
dnl             generate output
dnl ======================================================================

AC_CONFIG_FILES([GNUmakefile \
                 core/GNUmakefile \
                 host/GNUmakefile \
                 prog/GNUmakefile \
         config/GNUmakefile \
         config/kconfig/GNUmakefile \
         config/kconfig/lxdialog/GNUmakefile
                ])
AC_OUTPUT

dnl ======================================================================
dnl generate rack_config.h
dnl ======================================================================

#
# generate core/usb4rt_config.h
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

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

AC_C_INLINE
AC_HEADER_STDC
AC_CHECK_FUNCS([memset])

dnl ============================================================================
dnl  checking arch
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

dnl ============================================================================
dnl  ARG-Enable Common-Debug (default=enabled)
dnl ============================================================================

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

dnl ============================================================================
dnl  ARG-Enable Time-Debug (default=disabled)
dnl ============================================================================

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

dnl ============================================================================
dnl  ARG-Enable Transfer-Descriptor-Debug (default=disabled)
dnl ============================================================================

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

dnl ============================================================================
dnl  ARG-Enable Queue-Head-Debug (default=disabled)
dnl ============================================================================

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

dnl ============================================================================
dnl  ARG-Enable Bandwidth-Reclamation (default=enabled)
dnl ============================================================================

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

dnl ============================================================================
dnl  xenomai-path
dnl ============================================================================

AC_ARG_WITH(xeno-dir,
  AS_HELP_STRING([--with-xeno-dir=<XENOMAI>], [path to installation directory of xenomai ]),
    [case "${withval}" in
    "" | y | ye | yes | n | no)
            AC_MSG_ERROR([You must supply an argument to --with-xeno-dir.])
        ;;
    esac
    CONFIG_USB4RT_XENOMAIDIR="$withval"
    ])

dnl ============================================================================
dnl  try to find xenomai
dnl ============================================================================

AC_MSG_CHECKING([for Xenomai installation])
XENO_CONFIG="${CONFIG_USB4RT_XENOMAIDIR}/bin/xeno-config"
if test -x "${XENO_CONFIG}"; then
  XENO_DIR="`cd ${CONFIG_USB4RT_XENOMAIDIR} && pwd`" # let the shell normalize the path
  AC_MSG_RESULT([${XENO_DIR}])
else
  AC_MSG_RESULT([no Xenomai found in $CONFIG_USB4RT_XENOMAIDIR])
fi

dnl ============================================================================
dnl  get xenomai prefix
dnl ============================================================================

AC_MSG_CHECKING([for Xenomai PREFIX])
USB4RT_XENO_PREFIX="`cd \`${XENO_CONFIG} --prefix\` && pwd`"     # let the shell normalize the path
AC_MSG_RESULT([${USB4RT_XENO_PREFIX}])

AC_MSG_CHECKING([for Xenomai DESTDIR])
if test "${USB4RT_XENO_PREFIX}" \!= "${XENO_DIR}"; then
  USB4RT_XENO_DESTDIR="`cd \`echo ${XENO_DIR} | sed -e s,${USB4RT_XENO_PREFIX}$,,\` && pwd`"
  export DESTDIR="${USB4RT_XENO_DESTDIR}"

  USB4RT_XENO_PREFIX="`cd \`${XENO_CONFIG} --prefix\` && pwd`"   # let the shell normalize the path
  if test "${USB4RT_XENO_PREFIX}" = "${XENO_DIR}"; then
    AC_MSG_RESULT([${DESTDIR}])
  else
    AC_MSG_ERROR([*** Cannot identify DESTDIR])
  fi
else
  AC_MSG_RESULT([none])
fi

AC_SUBST(USB4RT_XENO_PREFIX)

dnl ============================================================================
dnl  checking Xenomai version
dnl ============================================================================

AC_MSG_CHECKING([for Xenomai version])
CONFIG_USB4RT_XENO_VERSION_PRE="`${XENO_CONFIG} --version`"

case "$CONFIG_USB4RT_XENO_VERSION_PRE" in
  2.0 | 2.0.*)
    CONFIG_USB4RT_XENO_VERSION=xeno-2x
    AC_DEFINE(CONFIG_XENO_2X, 1, [We build for Xenoami 2.0 or better])
    ;;
  *)
    AC_MSG_ERROR([*** Unsupported Xenomai version $CONFIG_USB4RT_XENO_VERSION_PRE in $XENO_DIR])
  ;;
esac
AC_MSG_RESULT([${CONFIG_USB4RT_XENO_VERSION_PRE}])
AM_CONDITIONAL(CONFIG_USB4RT_XENO_VERSION,[test "$CONFIG_USB4RT_XENO_VERSION" = xeno-2x])

dnl ============================================================================
dnl  checking xenomai config file
dnl ============================================================================

AC_MSG_CHECKING([for Xenomai config file])
CONFIG_USB4RT_XENO_CONFIG_FILE="`${XENO_CONFIG} --config`"
if test \! -f $CONFIG_USB4RT_XENO_CONFIG_FILE; then
  AC_MSG_ERROR([*** Xenoami configuration file not found])
fi
AC_MSG_RESULT([${CONFIG_USB4RT_XENO_CONFIG_FILE}])


# source (load) the Xenomai configuration
. ${CONFIG_USB4RT_XENO_CONFIG_FILE}

dnl ============================================================================
dnl  checking xenomai skins
dnl ============================================================================

AC_MSG_CHECKING([for Xenomai native skin])
    case "${CONFIG_XENO_SKIN_NATIVE}" in
    y) AC_MSG_RESULT([${CONFIG_XENO_SKIN_NATIVE}]) ;;
        *) AC_MSG_ERROR([*** Please enable native skin]) ;;
    esac

dnl ============================================================================
dnl  checking xenomai skins
dnl ============================================================================

XENO_USER_CFLAGS="`${XENO_CONFIG} --xeno-cflags`"

AC_MSG_CHECKING([for Xenomai module cflags])
XENO_KMOD_CFLAGS="`${XENO_CONFIG} --module-cflags`"
AC_MSG_RESULT([${XENO_KMOD_CFLAGS}])

CONFIG_USB4RT_XENO_ARCH="`${XENO_CONFIG} --arch`"
XENO_LIBRARIES="`${XENO_CONFIG} --library-dir`"
XENO_SYMBOLS="`${XENO_CONFIG} --symbol-dir`"

dnl ======================================================================
dnl             create links to Xenomai symbols
dnl ======================================================================

case "${CONFIG_USB4RT_XENO_VERSION}" in
  xeno-2x)
    mkdir -p symbols
    rm -rf symbols/*
    for FILE in `ls ${XENO_SYMBOLS}/*`; do
    ${LN_S} ${FILE} symbols/$(basename ${FILE})
    done
    ;;
esac


dnl ============================================================================
dnl   architecture tests....
dnl ============================================================================

AC_MSG_CHECKING([for Xenomai target arch])
AC_MSG_RESULT([${CONFIG_USB4RT_XENO_ARCH}])

AC_MSG_CHECKING([for USB4RT target arch])
AC_MSG_RESULT([${USB4RT_TARGET_ARCH}])

if test x"${USB4RT_TARGET_ARCH}" != x"${CONFIG_USB4RT_XENO_ARCH}"; then
    AC_MSG_ERROR([*** Xenomai and USB4RT target architectures are not the same])
fi

case "${CONFIG_USB4RT_XENO_ARCH}" in
  i386)
    AC_DEFINE(CONFIG_ARCH_I386, 1,[we are building for x86])
  ;;
  *)
    AC_MSG_ERROR([*** unsupported architecture])
    ;;
esac
AC_SUBST(USB4RT_TARGET_ARCH)

dnl ============================================================================
dnl  get linux kernel tree and version
dnl ============================================================================

XENO_LINUX_DIR="`${XENO_CONFIG} --linux-dir`"
XENO_LINUX_VERSION="`${XENO_CONFIG} --linux-version`"

AC_MSG_CHECKING([for Linux source tree])
if test \! -r ${XENO_LINUX_DIR}/Makefile; then
   echo
   AC_MSG_ERROR([*** No Linux kernel tree in ${XENO_LINUX_DIR}])
fi

if test \! -r ${XENO_LINUX_DIR}/.config; then
   echo
   AC_MSG_ERROR([*** $XENO_LINUX_DIR is not configured])
fi

AC_SUBST(XENO_LINUX_DIR)

dnl ============================================================================
dnl  check linux version
dnl ============================================================================

case "${XENO_LINUX_VERSION}" in
  2.6.*)
    ;;
  *)
    AC_MSG_ERROR([*** Unsupported kernel version $XENO_LINUX_VERSION])
    ;;
esac

AC_SUBST(USB4RT_MODULE_EXT)
AM_CONDITIONAL(CONFIG_KBUILD,[test "$CONFIG_KBUILD" = y])


dnl ============================================================================
dnl  some linux config checks
dnl ============================================================================

. ${XENO_LINUX_DIR}/.config
AC_MSG_RESULT([$XENO_LINUX_DIR (kernel ${XENO_LINUX_VERSION}${CONFIG_SMP:+-SMP})])

if test x${CONFIG_MODULES} = x; then
  AC_MSG_ERROR([*** Please set CONFIG_MODULES for $XENO_LINUX_DIR and rebuild your kernel])
fi

if test x${CONFIG_MODVERSIONS} = xy; then
  AC_MSG_ERROR([*** Please unset CONFIG_MODVERSIONS for $XENO_LINUX_DIR and rebuild your kernel])
fi

dnl ============================================================================
dnl  Check for an interrupt pipeline extension.
dnl ============================================================================

AC_MSG_CHECKING([for interrupt pipeline extension])

if test -r $XENO_LINUX_DIR/include/linux/ipipe.h; then
   v=`grep '^#define.*IPIPE_ARCH_STRING.*"' $XENO_LINUX_DIR/include/asm-$USB4RT_TARGET_ARCH/ipipe.h|sed -e 's,.*"\(.*\)"$,\1,'`
   AC_MSG_RESULT([Adeos newgen (I-pipe $v)])
   if test x${CONFIG_IPIPE} = x; then
      AC_MSG_ERROR([Please activate the Adeos support in $XENO_LINUX_DIR and rebuild your kernel])
   fi
   if test x${CONFIG_IPIPE_EXTENDED} = x; then
      AC_MSG_ERROR([Please re-patch your kernel with the Adeos support])
   fi
elif test -r $XENO_LINUX_DIR/include/linux/adeos.h; then
   v=`grep '^#define.*ADEOS_ARCH_STRING.*"' $XENO_LINUX_DIR/include/asm-$USB4RT_TARGET_ARCH/adeos.h|sed -e 's,.*"\(.*\)"$,\1,'`
   AC_MSG_RESULT([Adeos oldgen ($v)])
   if test x${CONFIG_ADEOS_CORE} = x; then
      AC_MSG_ERROR([Please activate the Adeos support in $XENO_LINUX_DIR and rebuild your kernel])
   fi
else
   AC_MSG_ERROR([No interrupt pipeline extension enabled in your kernel.
    You need to apply an Adeos patch first.])
fi

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


dnl ============================================================================
dnl  crosscompiling
dnl ============================================================================

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
  \$(MAKE) -C $XENO_LINUX_DIR                             \
  CROSS_COMPILE=\$(CROSS_COMPILE)                         \
    ARCH=\$(USB4RT_TARGET_ARCH)                           \
    M=\"\`pwd\`\"                                         \
    V=\$(V)                                               \
    xeno_dir=\"${USB4RT_XENO_PREFIX}\"                    \
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
bs_kcompile="make -C $XENO_LINUX_DIR ARCH=$USB4RT_TARGET_ARCH CROSS_COMPILE=$CROSS_COMPILE V=1 M=`pwd` SUBDIRS=`pwd` modules"

#
# Xenomai header checks
#

ac_save_CPPFLAGS="${CPPFLAGS}"
CPPFLAGS="${ac_save_CPPFLAGS} ${XENO_KMOD_CFLAGS}"

# Working around some Autoconf ugliness here: names of variables
# stored into the cache are canonicalized, turning /'s into _'s. So if
# you need to match X_Y.h and X/Y.h as different filenames, you are
# toast. Whichever comes first will be cached as "X_Y_h" and the
# second one will never get checked in the fs, but simply
# uncached. This applies to native_sem.h and native/sem.h for instance.
# To prevent this, we must only check a single set of non-conflicting
# headers, depending on the Xenomai version which has been
# identified. (rpm)

BS_CHECK_KHEADERS([native/sem.h],
      [],
      [AC_MSG_ERROR([*** header not found or working, please check Xenomai installation])],
      [
        #include <linux/kernel.h>
      ])
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

if test \! -r .usb4rt_config; then
   set | grep -e '^CONFIG_USB4RT' -e '^CONFIG_MODULES' | \
   sed -e 's,\(.*\)=\(.*\)$,\1=\"\2\",' > .usb4rt_config
fi

##########################
# Output files.
##########################
AC_CONFIG_FILES([GNUmakefile \
                 core/GNUmakefile \
                 host/GNUmakefile \
                 prog/GNUmakefile \
         config/GNUmakefile \
         config/kconfig/GNUmakefile \
         config/kconfig/lxdialog/GNUmakefile
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

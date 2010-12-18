
%define pfx /opt/freescale/rootfs/%{_target_cpu}

Summary         : platform specific unit tests for mxc platform
Name            : imx-lib-dirbuild
Version         : 1
Release         : 1
License         : GPL/Other
Vendor          : Freescale
Packager        : Rob Herring
Group           : Test
BuildRoot       : %{_tmppath}/%{name}
Prefix          : %{pfx}

%Description
%{summary}

%Prep

%Build
if [ -z "$PKG_KERNEL_KBUILD_PRECONFIG" ]
then
      KERNELDIR="$PWD/../linux"
      KBUILD_OUTPUT="$PWD/../linux"
else
      KERNELDIR="$PKG_KERNEL_PATH_PRECONFIG"
      KBUILD_OUTPUT="$(eval echo ${PKG_KERNEL_KBUILD_PRECONFIG})"
fi

if [ -z "$IMX_LIB_DIR" ]
then
	echo IMX_LIB_DIR must be set
	exit 1
fi
cd $IMX_LIB_DIR

PLATFORM_UPPER="$(echo $PLATFORM | awk '{print toupper($0)}')"

# Build libraries
INCLUDE="-I$DEV_IMAGE/usr/src/linux/include \
-I$KERNELDIR/drivers/mxc/security/rng/include \
-I$KERNELDIR/drivers/mxc/security/sahara2/include"
make -j1 PLATFORM=$PLATFORM_UPPER INCLUDE="$INCLUDE" all

%Install
if [ -z "$PKG_KERNEL_KBUILD_PRECONFIG" ]
then
      KERNELDIR="$PWD/../linux"
      KBUILD_OUTPUT="$PWD/../linux"
else
      KERNELDIR="$PKG_KERNEL_PATH_PRECONFIG"
      KBUILD_OUTPUT="$(eval echo ${PKG_KERNEL_KBUILD_PRECONFIG})"
fi

rm -rf $RPM_BUILD_ROOT
#mkdir -p $RPM_BUILD_ROOT/%{pfx}/unit_tests

if [ -z "$IMX_LIB_DIR" ]
then
	echo IMX_LIB_DIR must be set
	exit 1
fi
cd $IMX_LIB_DIR

# install libraries and headers
make DEST_DIR=$RPM_BUILD_ROOT/%{pfx} install

%Clean
rm -rf $RPM_BUILD_ROOT

%Files
%defattr(755,root,root)
%{pfx}/*

Summary: Libraries and programs for mobile robot SLAM and navigation
Name: mrpt
Version: 0.7.0
Release: 0.1.20090529svn1047%{?dist}
License: GPLv3+
Group: Development/Libraries
URL: http://babel.isa.uma.es/mrpt/

# Tarballs at http://babel.isa.uma.es/mrpt/src-repo/ are the same that those
# at SourceForge (http://downloads.sourceforge.net/mrpt/mrpt-%{version}.tar.gz) 
# but without the directory "otherlibs/sift-hess", which contains code with a 
# patent pending for approval.

# The source for this package was pulled from upstream's vcs.  Use the
# following commands to generate the tarball:
#  svn export http://babel.isa.uma.es/mrpt-browse-code/mrpt-0.7.0 mrpt-0.7.0
#  tar -czvf mrpt-0.7.0-20090529svn1047.tar.gz mrpt-0.70
Source: http://babel.isa.uma.es/mrpt/src-repo/mrpt-0.7.0-20090529svn1047.tar.gz

BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

BuildRequires: cmake
BuildRequires: wxGTK-devel, opencv-devel, freeglut-devel, lib3ds-devel
BuildRequires: boost-devel
BuildRequires: doxygen, ghostscript
BuildRequires: tex(latex), tex(dvips)
BuildRequires: libdc1394-devel
BuildRequires: libftdi-devel, libusb-devel
BuildRequires: libjpeg-devel
BuildRequires: desktop-file-utils

%description
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

The libraries include classes for easily managing 3D(6D) geometry, 
probability density functions (pdfs) over many predefined variables (points 
and poses, landmarks, maps), Bayesian inference (Kalman filters, particle 
filters), image processing, path planning and obstacle avoidance, 3D 
visualization of all kind of maps (points, occupancy grids, landmarks,...), 
etc.
Gathering, manipulating and inspecting very large robotic datasets (Rawlogs)
efficiently is another goal of MRPT, supported by several classes and 
applications.

The MRPT is free software and is released under the GPL. 


# Subpackages "ann", "aria", "core", "hwdrivers", and "reactivenav" are provided
# in order to minimize dependencies of future packages that might depend on a
# subset only of all the MRPT libraries. The subpackage "libs" can be used as 
# a shortcut for all the libraries.

%package ann
Summary: Mobile Robot Programming Toolkit - Approximate Nearest Neighbor library
Group: Development/Libraries
%description ann
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides the MRPT built-in ANN (Approximate Nearest Neighbor) 
library, by David M. Mount  and Sunil Arya.

%package aria
Summary: Mobile Robot Programming Toolkit - ActiveMedia's ARIA library
Group: Development/Libraries
%description aria
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides ActiveMedia's ARIA library for control of their robotic 
mobile bases.


%package core
Summary: Mobile Robot Programming Toolkit - The core library
Group: Development/Libraries
Requires: %{name}-ann = %{version}-%{release}
%description core
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides the mrpt-core library, which includes classes related to
mathematic, slam, opengl, geometry, etc.

%package hwdrivers
Summary: Mobile Robot Programming Toolkit - Sensor interfaces library
Group: Development/Libraries
Requires: %{name}-core = %{version}-%{release}
Requires: %{name}-aria = %{version}-%{release}
%description hwdrivers
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides the mrpt-hwdrivers library, including classes for serial
ports, FTDI USB chips, SICK and HOKUYO laser scanners, etc.

%package reactivenav
Summary: Mobile Robot Programming Toolkit - Reactive Navigation library
Group: Development/Libraries
Requires: %{name}-core = %{version}-%{release}
%description reactivenav
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides the mrpt-reactivenav library, with implementation of 
reactive navigation algorithms (VFF, ND, PTG-based space transformations).


%package apps
Summary: Mobile Robot Programming Toolkit - Console and GUI applications
Group: Applications/Engineering
Requires: %{name}-core = %{version}-%{release}
Requires: %{name}-reactivenav = %{version}-%{release}
Requires: %{name}-hwdrivers = %{version}-%{release}
%description apps
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides a set of console and GUI applications for manipulating 
datasets, particle filtering localization and SLAM, grabbing data from 
robotic sensors, etc.


%package libs
Summary: Mobile Robot Programming Toolkit - All the libraries
Group: Development/Libraries
Requires: %{name}-core = %{version}-%{release}
Requires: %{name}-reactivenav = %{version}-%{release}
Requires: %{name}-hwdrivers = %{version}-%{release}
%description libs
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This virtual package depends on all MRPT libraries.


%package devel
Summary: Mobile Robot Programming Toolkit - Development package
Group: Development/Libraries
Requires: %{name}-libs = %{version}-%{release}
# wxGTK-devel is needed due to headers dependencies.
Requires: wxGTK-devel, pkgconfig
%description devel
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides the headers and required files to build third-party 
applications that use MRPT libraries.


%package doc
Summary: Mobile Robot Programming Toolkit - Documentation
Group: Documentation
%description doc
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package contains documentation, examples and the reference generated
with Doxygen.


%prep
%setup -q
# Fix encoding of the mrpt-book file
gzip -d doc/mrpt-book.ps.gz
iconv -f ISO8859-1 -t UTF-8 doc/mrpt-book.ps > mrpt-book.ps.conv
mv -f mrpt-book.ps.conv doc/mrpt-book.ps
gzip doc/mrpt-book.ps


%build
# The flag CMAKE_MRPT_IS_RPM_PACKAGE disables global "-mtune=native"
%cmake . -DCMAKE_MRPT_IS_RPM_PACKAGE=1
make VERBOSE=1 %{?_smp_mflags}
make documentation_html
make man_pages_all

%check
export LD_LIBRARY_PATH=$(pwd)/lib
make test VERBOSE=1 ARGS="-VV"

%install
rm -rf $RPM_BUILD_ROOT
make install DESTDIR=$RPM_BUILD_ROOT
# Validate .g files:
find ${RPM_BUILD_ROOT}%{_datadir}/applications/ -name "*.desktop" | xargs -I FIL desktop-file-validate FIL

%clean
rm -rf $RPM_BUILD_ROOT

%files ann
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-ann.so.*

%files aria
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-aria.so.*

%files core
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-core.so.*
# This directory is empty here but contains files in other sub-packages 
#  depending on mrpt-core:
%dir %{_datadir}/mrpt
%{_datadir}/mime/packages/*.xml

%files hwdrivers
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-hwdrivers.so.*

%files reactivenav
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-reactivenav.so.*


%files apps
%defattr(-,root,root,-)
%doc README COPYING
%{_bindir}/*
%{_datadir}/applications/*.desktop
%{_datadir}/pixmaps/*.ico
%{_datadir}/pixmaps/*.xpm
%{_datadir}/mime/packages/*.xml
# %{_datadir}/mrpt is owned by mrpt-core:
%{_datadir}/mrpt/config_files/
%{_datadir}/mrpt/datasets/
%{_mandir}/man1/*


%files devel
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/*.so
%{_includedir}/mrpt
%{_libdir}/pkgconfig/libmrpt.pc
# %{_datadir}/mrpt is owned by mrpt-core:
%{_datadir}/mrpt/MRPTConfig.cmake


%files doc
%defattr(-,root,root,-)
#%doc README COPYING
%dir %{_datadir}/doc/mrpt-doc/
%{_datadir}/doc/mrpt-doc/*


%files libs
%defattr(-,root,root,-)
%doc README COPYING


%ifos linux
%post ann -p /sbin/ldconfig
%postun ann -p /sbin/ldconfig

%post aria -p /sbin/ldconfig
%postun aria -p /sbin/ldconfig

%post hwdrivers -p /sbin/ldconfig
%postun hwdrivers -p /sbin/ldconfig

%post reactivenav -p /sbin/ldconfig
%postun reactivenav -p /sbin/ldconfig

%post apps
update-desktop-database &> /dev/null || :

%postun apps
update-desktop-database &> /dev/null || :

%post core
/sbin/ldconfig
update-mime-database %{_datadir}/mime &> /dev/null || :

%postun core
/sbin/ldconfig
update-mime-database %{_datadir}/mime &> /dev/null || :

%endif


%changelog
* Tue Jul 14 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.7.0-0.1.20090529svn1047
- Packaging of new upstream version 0.7.0.

* Wed Feb 25 2009 Fedora Release Engineering <rel-eng@lists.fedoraproject.org> - 0.6.5-0.4.20090213svn807
- Rebuilt for https://fedoraproject.org/wiki/Fedora_11_Mass_Rebuild

* Thu Feb 19 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.6.5-0.3.20090213svn807
- Fixed ownship of datadir/mrpt/config_files/ by two sub-packages.

* Sat Feb 13 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.6.5-0.2.20090213svn807
- All applications are now in mrpt-apps.

* Sat Feb 13 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.6.5-0.1.20090213
- New upstream sources.
- Individual packages created for each MRPT application.
- Removed unneeded dependencies from -devel package.
- Fixed "doc" package should own the mrpt-doc directory.
- Mime types moved to mrpt-core package.

* Sun Jan 18 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.6.5-0.1.20090118svn746
- New upstream sources.
- Fixed license tag to "GPLv3+".
- Added "export LD_LIBRARY_PATH..." at "check" to allow the tests to work.
- Comments added explaining the split in subpackages.
- devel package depends on wxGTK-devel instead of wxGTK due to needed headers.
- datadir/mrpt is now owned by mrpt-core to avoid duplicated ownership.
- Several fixes to libmrpt.pc
- Added calls to "update-desktop-database" and "update-mime-database" in post/postun of mrpt-apps.
- Corrected texlive-latex dependency to enable compilation of doxygen formulas.

* Sun Jan 8 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.6.4-2
- More verbose output in 'make test', and possibly fixed wrong compiler flag.
- Fixed ownership of the same file MRPTConfig.cmake in two subpackages.

* Sun Jan 4 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.6.4-1
- Initial packaging for Fedora.


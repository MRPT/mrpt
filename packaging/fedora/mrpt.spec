Summary: Libraries and programs for mobile robot SLAM and navigation
Name: mrpt
Version: 0.9.5
Release: 0.1.20110916svn2655%{?dist}
License: GPLv3+
Group: Development/Libraries
URL: https://www.mrpt.org/

# Tarballs at http://babel.isa.uma.es/mrpt/src-repo/ are the same that those
# at SourceForge (http://downloads.sourceforge.net/mrpt/mrpt-%{version}.tar.gz) 
# but without the directory "otherlibs/sift-hess", which contains code with a 
# patent pending for approval.

# The source for this package was pulled from upstream's vcs.  Use the
# following commands to generate the tarball:
#  svn export http://babel.isa.uma.es/mrpt-browse-code/mrpt-0.7.0 mrpt-0.7.0
#  tar -czvf mrpt-0.7.0-20090529svn1047.tar.gz mrpt-0.70
Source: http://archive.mrpt.org/fedora-packages/mrpt-0.9.5-20110916svn2655.tar.gz

BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

BuildRequires: cmake
BuildRequires: wxGTK-devel, opencv-devel, freeglut-devel, lib3ds-devel
BuildRequires: boost-devel
BuildRequires: doxygen, ghostscript, graphviz
BuildRequires: tex(latex), tex(dvips)
BuildRequires: libdc1394-devel
BuildRequires: libftdi-devel, libusb-devel
BuildRequires: libjpeg-devel
BuildRequires: libusb1-devel
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

%package base
Summary: Mobile Robot Programming Toolkit - mrpt-base
Group: Development/Libraries
%description base
The Mobile Robot Programming Toolkit (MRPT) library mrpt-base


%package opengl
Summary: Mobile Robot Programming Toolkit - mrpt-opengl
Group: Development/Libraries
Requires: %{name}-base = %{version}-%{release}
%description opengl
The Mobile Robot Programming Toolkit (MRPT) library mrpt-opengl

%package bayes
Summary: Mobile Robot Programming Toolkit - mrpt-bayes
Group: Development/Libraries
Requires: %{name}-base = %{version}-%{release}
%description bayes
The Mobile Robot Programming Toolkit (MRPT) library mrpt-bayes

%package graphs
Summary: Mobile Robot Programming Toolkit - mrpt-graphs
Group: Development/Libraries
Requires: %{name}-base = %{version}-%{release}
%description graphs
The Mobile Robot Programming Toolkit (MRPT) library mrpt-graphs

%package gui
Summary: Mobile Robot Programming Toolkit - mrpt-gui
Group: Development/Libraries
Requires: %{name}-opengl = %{version}-%{release}
%description gui
The Mobile Robot Programming Toolkit (MRPT) library mrpt-gui


%package obs
Summary: Mobile Robot Programming Toolkit - mrpt-obs
Group: Development/Libraries
Requires: %{name}-opengl = %{version}-%{release}
%description obs
The Mobile Robot Programming Toolkit (MRPT) library mrpt-obs


%package scanmatching
Summary: Mobile Robot Programming Toolkit - mrpt-scanmatching
Group: Development/Libraries
Requires: %{name}-base = %{version}-%{release}
%description scanmatching
The Mobile Robot Programming Toolkit (MRPT) library mrpt-scanmatching


%package topography
Summary: Mobile Robot Programming Toolkit - mrpt-topography
Group: Development/Libraries
Requires: %{name}-scanmatching = %{version}-%{release}
%description topography
The Mobile Robot Programming Toolkit (MRPT) library mrpt-topography


%package maps
Summary: Mobile Robot Programming Toolkit - mrpt-maps
Group: Development/Libraries
Requires: %{name}-obs = %{version}-%{release}
%description maps
The Mobile Robot Programming Toolkit (MRPT) library mrpt-maps


%package vision
Summary: Mobile Robot Programming Toolkit - mrpt-vision
Group: Development/Libraries
Requires: %{name}-obs = %{version}-%{release}
%description vision
The Mobile Robot Programming Toolkit (MRPT) library mrpt-vision


%package hwdrivers
Summary: Mobile Robot Programming Toolkit - mrpt-hwdrivers
Group: Development/Libraries
Requires: %{name}-obs = %{version}-%{release}
Requires: %{name}-gui = %{version}-%{release}
%description hwdrivers
The Mobile Robot Programming Toolkit (MRPT) library mrpt-hwdrivers


%package slam
Summary: Mobile Robot Programming Toolkit - mrpt-slam
Group: Development/Libraries
Requires: %{name}-bayes = %{version}-%{release}
Requires: %{name}-scanmatching = %{version}-%{release}
Requires: %{name}-maps = %{version}-%{release}
Requires: %{name}-vision = %{version}-%{release}
%description slam
The Mobile Robot Programming Toolkit (MRPT) library mrpt-slam

%package graphslam
Summary: Mobile Robot Programming Toolkit - mrpt-graphslam
Group: Development/Libraries
Requires: %{name}-slam = %{version}-%{release}
Requires: %{name}-graphs = %{version}-%{release}
%description graphslam
The Mobile Robot Programming Toolkit (MRPT) library mrpt-graphslam


%package reactivenav
Summary: Mobile Robot Programming Toolkit - mrpt-reactivenav
Group: Development/Libraries
Requires: %{name}-maps = %{version}-%{release}
%description reactivenav
The Mobile Robot Programming Toolkit (MRPT) library mrpt-reactivenav


%package detectors
Summary: Mobile Robot Programming Toolkit - mrpt-detectors
Group: Development/Libraries
Requires: %{name}-maps = %{version}-%{release}
Requires: %{name}-gui = %{version}-%{release}
Requires: %{name}-vision = %{version}-%{release}
%description detectors
The Mobile Robot Programming Toolkit (MRPT) library mrpt-detectors


%package hmtslam
Summary: Mobile Robot Programming Toolkit - mrpt-hmtslam
Group: Development/Libraries
Requires: %{name}-slam = %{version}-%{release}
%description hmtslam
The Mobile Robot Programming Toolkit (MRPT) library mrpt-hmtslam


%package libs
Summary: Mobile Robot Programming Toolkit - All the libraries
Group: Development/Libraries
Requires: %{name}-detectors = %{version}-%{release}
Requires: %{name}-reactivenav = %{version}-%{release}
Requires: %{name}-hmtslam = %{version}-%{release}
Requires: %{name}-slam = %{version}-%{release}
Requires: %{name}-topography = %{version}-%{release}
Requires: %{name}-gui = %{version}-%{release}
Requires: %{name}-obs = %{version}-%{release}
Requires: %{name}-maps = %{version}-%{release}
Requires: %{name}-opengl = %{version}-%{release}
Requires: %{name}-base = %{version}-%{release}
Requires: %{name}-bayes = %{version}-%{release}
Requires: %{name}-graphs = %{version}-%{release}
Requires: %{name}-graphslam = %{version}-%{release}
%description libs
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This virtual package depends on all MRPT libraries.



%package apps
Summary: Mobile Robot Programming Toolkit - Console and GUI applications
Group: Applications/Engineering
Requires: %{name}-libs = %{version}-%{release}
%description apps
The Mobile Robot Programming Toolkit (MRPT) is an extensive, cross-platform,
and open source C++ library aimed to help robotics researchers to design and
implement algorithms in the fields of Simultaneous Localization and Mapping 
(SLAM), computer vision, and motion planning (obstacle avoidance).

This package provides a set of console and GUI applications for manipulating 
datasets, particle filtering localization and SLAM, grabbing data from 
robotic sensors, etc.



%package devel
Summary: Mobile Robot Programming Toolkit - Development package
Group: Development/Libraries
Requires: %{name}-libs = %{version}-%{release}
Requires: pkgconfig
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
#gzip -d doc/mrpt-book.ps.gz
#iconv -f ISO8859-1 -t UTF-8 doc/mrpt-book.ps > mrpt-book.ps.conv
#mv -f mrpt-book.ps.conv doc/mrpt-book.ps
#gzip doc/mrpt-book.ps


%build
# The flag CMAKE_MRPT_IS_RPM_PACKAGE disables global "-mtune=native"
%cmake . -DCMAKE_MRPT_IS_RPM_PACKAGE=1
make VERBOSE=1 %{?_smp_mflags}
make documentation_html
make man_pages_all

%check
export LD_LIBRARY_PATH=$(pwd)/lib
make test VERBOSE=1 ARGS="-VV" || echo "**Warning**: unit tests failed, check whether it was only due to SSE* stuff" 

%install
rm -rf $RPM_BUILD_ROOT
make install DESTDIR=$RPM_BUILD_ROOT
# Validate .g files:
find ${RPM_BUILD_ROOT}%{_datadir}/applications/ -name "*.desktop" | xargs -I FIL desktop-file-validate FIL

%clean
rm -rf $RPM_BUILD_ROOT

%files base
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-base.so.*
# This directory is empty here but contains files in other sub-packages 
#  depending on mrpt-base:
%dir %{_datadir}/mrpt
%{_datadir}/mime/packages/*.xml

%files opengl
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-opengl.so.*

%files scanmatching
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-scanmatching.so.*

%files bayes
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-bayes.so.*

%files graphs
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-graphs.so.*

%files obs
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-obs.so.*

%files gui
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-gui.so.*

%files topography
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-topography.so.*

%files maps
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-maps.so.*

%files vision
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-vision.so.*

%files hwdrivers
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-hwdrivers.so.*

%files reactivenav
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-reactivenav.so.*

%files detectors
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-detectors.so.*

%files slam
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-slam.so.*

%files graphslam
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-graphslam.so.*

%files hmtslam
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/libmrpt-hmtslam.so.*


%files apps
%defattr(-,root,root,-)
%doc README COPYING
%{_bindir}/*
%{_datadir}/applications/*.desktop
%{_datadir}/pixmaps/*.ico
%{_datadir}/pixmaps/*.xpm
%{_datadir}/mime/packages/*.xml
# %{_datadir}/mrpt is owned by mrpt-base:
%{_datadir}/mrpt/config_files/
%{_datadir}/mrpt/datasets/
%{_mandir}/man1/*


%files devel
%defattr(-,root,root,-)
%doc README COPYING
%{_libdir}/*.so
%{_includedir}/mrpt
%{_libdir}/pkgconfig/*.pc
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
%post opengl -p /sbin/ldconfig
%postun opengl -p /sbin/ldconfig

%post gui -p /sbin/ldconfig
%postun gui -p /sbin/ldconfig

%post obs -p /sbin/ldconfig
%postun obs -p /sbin/ldconfig

%post scanmatching -p /sbin/ldconfig
%postun scanmatching -p /sbin/ldconfig

%post topography -p /sbin/ldconfig
%postun topography -p /sbin/ldconfig

%post maps -p /sbin/ldconfig
%postun maps -p /sbin/ldconfig

%post vision -p /sbin/ldconfig
%postun vision -p /sbin/ldconfig

%post hwdrivers -p /sbin/ldconfig
%postun hwdrivers -p /sbin/ldconfig

%post reactivenav -p /sbin/ldconfig
%postun reactivenav -p /sbin/ldconfig

%post detectors -p /sbin/ldconfig
%postun detectors -p /sbin/ldconfig

%post slam -p /sbin/ldconfig
%postun slam -p /sbin/ldconfig

%post hmtslam -p /sbin/ldconfig
%postun hmtslam -p /sbin/ldconfig

%post bayes -p /sbin/ldconfig
%postun bayes -p /sbin/ldconfig

%post graphs -p /sbin/ldconfig
%postun graphs -p /sbin/ldconfig

%post graphslam -p /sbin/ldconfig
%postun graphslam -p /sbin/ldconfig

%post apps
update-desktop-database &> /dev/null || :

%postun apps
update-desktop-database &> /dev/null || :

%post base
/sbin/ldconfig
update-mime-database %{_datadir}/mime &> /dev/null || :

%postun base
/sbin/ldconfig
update-mime-database %{_datadir}/mime &> /dev/null || :

%endif


%changelog

* Fri Sep 16 2011 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.5-0.1.20110916svn2655
- New 0.9.5 svn snapshot.

* Tue Aug 23 2011 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.5-0.1.20110823svn2634
- New 0.9.5 svn snapshot.

* Mon Jan 10 2011 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.4-0.1.20110110svn2383
- New 0.9.4 svn snapshot, with more secure unit tests for autobuilders.

* Mon Jan 10 2011 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.4-0.1.20110110svn2382
- Packaging of new upstream version 0.9.4 (svn snapshot)

* Mon Jan 10 2011 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.4-0.1.20110110svn2380
- Packaging of new upstream version 0.9.4 (svn snapshot)

* Sat Dec 25 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.4-0.1.20101225svn2354
- Packaging of new upstream version 0.9.4 (svn snapshot)

* Wed Jul 14 2010 Dan Horák <dan@danny.cz> - 0.9.0-0.5
- rebuilt against wxGTK-2.8.11-2

* Sun Jul 4 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.0-0.4
- Rebuild needed by new opencv.

* Sun Jun 6 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.0-0.3
- Changed source tarball name numbering.

* Sat Jun 5 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.0-0.2
- Fixed build against OpenCV.

* Fri Jun 4 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.9.0-0.1
- Packaging of new upstream version 0.9.0.

* Sat Mar  6 2010 - Thomas Spura <tomspur@fedoraproject.org> 0.8.0-0.3.20100102svn1398
- rebuild as requested in
  http://lists.fedoraproject.org/pipermail/devel/2010-March/132519.html

* Fri Jan 22 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.8.0-0.2.20100102svn1398
- Fixed dependencies in spec file.

* Thu Jan 21 2010 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.8.0-0.1.20100102svn1398
- Packaging of new upstream version 0.8.0.

* Mon Aug 18 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.7.1-0.1.20090818svn1148
- Packaging of new upstream version 0.7.1, patched.

* Mon Aug 17 2009 - Jose Luis Blanco <joseluisblancoc@gmail.com> 0.7.1-0.1.20090817svn1147
- Packaging of new upstream version 0.7.1.

* Sat Jul 25 2009 Fedora Release Engineering <rel-eng@lists.fedoraproject.org> - 0.7.0-0.2.20090529svn1047
- Rebuilt for https://fedoraproject.org/wiki/Fedora_12_Mass_Rebuild

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


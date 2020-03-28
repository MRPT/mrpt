# How to release a new version of MRPT

These notes are mostly for myself (J.L. Blanco), but hopefully they will be
useful to other future maintainers.

## 1) Changelog and git stuff

After the new version seems good to go:

 * Go to the root MRPT directory.
 * Edit `doc/doxygen-pages/changeLog_doc.h` to set the release date.
 * Do the final `git commit` to `develop` before the release.
 * Merge to `master`, then create the new tag. This follows the [git-flow pattern](https://nvie.com/posts/a-successful-git-branching-model/):

![Git-flow pattern](design_of_images/git-flow-pattern.png)

## 2) Create source packages

Run:

```
bash scripts/prepare_release.sh
bash scripts/prepare_debian.sh
```

The packages are in `$(HOME)/mrpt_release` and `$(HOME)/mrpt_debian`

Now for windows binary packages (this is to be automated via AppVeyor)
(see also the script: MRPT/scripts/automated_build_msvc_binary_package.bat)

 * Extract mrpt-x.y.z.zip
 * Run the script: automated_create_all_windows_MSVC_MinGW_build_dirs.bat

## 3) Debian packages

 * Go to `$HOME/mrpt_debian/mrpt-x.y.z./debian`
 * Edit changelog
 * Go to mrpt_debian
 * `debuild -S -sa`
 * Test with: `lintian *.changes`
 * Test build in Debian Unstable

```
sudo ARCH=amd64 DIST=sid pbuilder --update
sudo ARCH=amd64 DIST=sid pbuilder --build  MRPT*.dsc
```
 * Test all binary packages with: `lintian *.deb`

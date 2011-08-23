cd MRPT
bash scritpts/prepare_fedora.sh

...

Test it with koji: e.g:

rpmbuild -bs AAAA.spec
koji build dist-f14 --scratch  AAA.src.rpm

...

cd ~/fedpkg/mrpt/master
(update mrpt.spec as needed)
fedpkg new-sources PATH/mrpt-XXX.tar.gz
fedpkg diff
fedpkg commit -p  # commit and push
fedpkg build

....

In the other branches:

Update mrpt.spec

git commit  # or commit -a
git push 

fedpkg build
fedpkg update
BODHI_USER=foo fedpkg update


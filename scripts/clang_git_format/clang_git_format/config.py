import string

PROGNAME = "clang-format"
CLANG_FORMAT_VERSION = "8.0.0"
_last_dot = CLANG_FORMAT_VERSION.rfind(".")
CLANG_FORMAT_SHORT_VERSION = CLANG_FORMAT_VERSION[:_last_dot]

# URL location of the "cached" copy of clang-format to download
# for users which do not have clang-format installed
CLANG_FORMAT_HTTP_LINUX_CACHE = "https://s3.amazonaws.com/boxes.10gen.com/build/clang-format-3.8-rhel55.tar.gz"
CLANG_FORMAT_HTTP_DARWIN_CACHE = "https://s3.amazonaws.com/boxes.10gen.com/build/clang%2Bllvm-3.8.0-x86_64-apple-darwin.tar.xz"

CLANG_FORMAT_SOURCE_TAR_BASE = string.Template(
    "clang+llvm-$version-$tar_path/bin/" +
    PROGNAME)

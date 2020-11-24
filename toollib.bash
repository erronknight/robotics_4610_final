#
# Think of this file like a "bash library".
#
# We `source` it into other scripts to use.
#

BUILD_THREADS=4

function configure_component {
	comp=$1
	compobjdir=objdir.$comp
	mkdir $compobjdir
	cmakepath=$(realpath $comp/)
	(cd $compobjdir && cmake $cmakepath)
}

function build_component {
	comp=$1
	(cd objdir.$comp && make -j $BUILD_THREADS $comp)
}

function run_component {
	comp=$1
	shift 1
	(cd objdir.$comp && ./$comp $@)
}

function clean_component {
	comp=$1
	(cd objdir.$comp && make clean)
}

function purge_component {
	comp=$1
	rm -rv objdir.$comp
}

# CS4610 Final

## Building

I made some changes to the toplevel Makefile, just running `make` will do a full
clean build.

The optimal way to do it is to run `make cmake` to set up the build env first.
Then it's kinda annoying because we're doing two layers.  Once you've built this
then you can build the binaries using like `make kickerbot`.  If you use `-B` to
force a full build I don't think it'll do the right thing.  Once it's done it
should drop the two bot programs in the current directory.

The technically best way that also uses more cores is to do something like
`(cd build/ && make -j 8 goaliebot kickerbot && cp *bot ..)`, but that's overly
verbose.  You can modify the `THREADS` variable in the toplevel Makefile to
change it how many it tells the other make instance to use.

It was messed up before because the wildcard dependency wasn't checking all the
files if they had updated.  I could have just fixed that but I don't know the
syntax, it's better to let CMake figure out what it should be.

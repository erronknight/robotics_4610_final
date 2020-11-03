
# TTask

This is super specific for HW07, but the core functionality would be here:

* `common.{cpp,hxx}`
* `tasklib.{cpp,hxx}`
* `nav.{cpp,hxx}`

I was also planning on refactoring things so that tasks have `aistate` as a
field that's populated when we init the task so we don't have to pass it in
each tick, although that's not *that* big of a deal.

# additional target to perform cppcheck run, requires cppcheck

# get all project files
file(GLOB_RECURSE ALL_SOURCE_FILES ../ *.cc *.h)

# --library=
# avr.cfg  cppcheck-cfg.rng  gnu.cfg  gtk.cfg  microsoft_sal.cfg  posix.cfg  qt.cfg  sdl.cfg  std.cfg  windows.cfg
add_custom_target(
        cppcheck
        COMMAND /usr/bin/cppcheck
        --enable=warning,performance,portability,information,missingInclude
        --std=c++11
        --library=gnu.cfg
        --template="[{severity}][{id}] {message} {callstack} \(On {file}:{line}\)"
        --verbose
        --quiet
        ${ALL_SOURCE_FILES}
)

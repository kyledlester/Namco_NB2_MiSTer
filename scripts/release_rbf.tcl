# Quartus post-flow script (Namco_NB2.qsf, POST_FLOW_SCRIPT_FILE).
# After a full compile, copy output_files/<revision>.rbf to
# Releases/<revision>_YYYYMMDD.rbf, the dated name MiSTer and its updaters
# expect. MRAs name the core without the date (<rbf>Namco_NB2</rbf>) and
# MiSTer picks the newest dated file.
set flow     [lindex $quartus(args) 0]
set revision [lindex $quartus(args) 2]
if {$flow ne "compile"} { return }

set rbf [file join output_files "${revision}.rbf"]
if {![file exists $rbf]} {
    post_message -type warning "release_rbf: $rbf not found; nothing copied"
    return
}
file mkdir Releases
set dest [file join Releases "${revision}_[clock format [clock seconds] -format %Y%m%d].rbf"]
file copy -force $rbf $dest
post_message "release_rbf: copied $rbf to $dest"

@ARGV = qw(.) unless @ARGV;
use File::Find;
find sub { s/\t//g $File::Find::name, -d && '/', "\n" }, @ARGV;

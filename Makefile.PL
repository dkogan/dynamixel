use 5.010;
use strict;
use warnings;
use ExtUtils::MakeMaker;

WriteMakefile(
    NAME                => 'Device::Dynamixel',
    AUTHOR              => q{Dima Kogan <dkogan@cpan.org>},
    VERSION_FROM        => 'lib/Device/Dynamixel.pm',
    ABSTRACT_FROM       => 'lib/Device/Dynamixel.pm',
    ($ExtUtils::MakeMaker::VERSION >= 6.3002
      ? ('LICENSE'=> 'perl')
      : ()),
    PL_FILES            => {},
    PREREQ_PM => {
        'Test::More'  => 0,
	'Const::Fast' => 0
    },
    MIN_PERL_VERSION => '5.10',
    dist                => { COMPRESS => 'gzip -9f', SUFFIX => 'gz', },
    clean               => { FILES => 'Device-Dynamixel-*' },
);

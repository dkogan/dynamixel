package Device::Dynamixel;

use strict;
use warnings;
use List::Util qw(sum);
use feature qw(say);
use Const::Fast;

=head1 NAME

Device::Dynamixel - Simple control of Robotis Dynamixel servo motors

=head1 VERSION

Version 0.023

=cut

our $VERSION = '0.023';


=head1 SYNOPSIS

 use Device::Dynamixel;

 open my $pipe, '+<', '/dev/ttyUSB0' or die "Couldn't open pipe for reading and writing";

 my $dynamixel = Dynamixel->new($pipe, 5);
 while(<>)
 {
   $dynamixel->moveMotorTo_deg($_);
 }

=head1 DESCRIPTION

This is a simple module to communicate with Robotis Dynamixel servo motors. The
Dynamixel AX-12 motors have been tested to work with this module, but the others
should work also.

A daisy-chained series string of motors is connected to the host via a simple
serial connection. Each motor in the series has an 8-bit address. This address
is present in every command to address specific motors. The current
implementation of this module uses an object per motor, NOT an object per string
of motors. This may change in the future.

These motors communicate using a particular protocol, which is implemented by
this module. Commands are sent to the motor. A STATUS reply is sent back after
each command. This module handles construction and parsing of Dynamixel packets,
as well as the sending and receiving data when needed.

=head2 VARIABLES

The dynamixel broadcast address is accessible as
$Device::Dynamixel::BROADCAST_ADDR. The addresses of Dynamixel control registers
are available as %Device::Dynamixel::addresses. This allows the application to
send arbitrary commands to the motor. For example, to set the moving speed of
the motor to 100%:

 $dynamixel->writeMotor($Device::Dynamixel::addresses{Moving_Speed_L},
                        [0xFF, 0x03]); # 0x3FF is the top speed for the
                                       # Dynamixel AX-12

=cut




# Constants defined in the dynamixel docs
const our $BROADCAST_ADDR => 0xFE;
const my %instructions =>
  (PING       => 0x01,
   READ_DATA  => 0x02,
   WRITE_DATA => 0x03,
   REG_WRITE  => 0x04,
   ACTION     => 0x05,
   RESET      => 0x06,
   SYNC_WRITE => 0x83);

const our %addresses =>
  (ModelNumber_L             => 0,
   ModelNumber_H             => 1,
   Version_of_Firmware       => 2,
   ID                        => 3,
   Baud_Rate                 => 4,
   Return_Delay_Time         => 5,
   CW_Angle_Limit_L          => 6,
   CW_Angle_Limit_H          => 7,
   CCW_Angle_Limit_L         => 8,
   CCW_Angle_Limit_H         => 9,
   Highest_Limit_Temperature => 11,
   Lowest_Limit_Voltage      => 12,
   Highest_Limit_Voltage     => 13,
   Max_Torque_L              => 14,
   Max_Torque_H              => 15,
   Status_Return_Level       => 16,
   Alarm_LED                 => 17,
   Alarm_Shutdown            => 18,
   Down_Calibration_L        => 20,
   Down_Calibration_H        => 21,
   Up_Calibration_L          => 22,
   Up_Calibration_H          => 23,
   Torque_Enable             => 24,
   LED                       => 25,
   CW_Compliance_Margin      => 26,
   CCW_Compliance_Margin     => 27,
   CW_Compliance_Slope       => 28,
   CCW_Compliance_Slope      => 29,
   Goal_Position_L           => 30,
   Goal_Position_H           => 31,
   Moving_Speed_L            => 32,
   Moving_Speed_H            => 33,
   Torque_Limit_L            => 34,
   Torque_Limit_H            => 35,
   Present_Position_L        => 36,
   Present_Position_H        => 37,
   Present_Speed_L           => 38,
   Present_Speed_H           => 39,
   Present_Load_L            => 40,
   Present_Load_H            => 41,
   Present_Voltage           => 42,
   Present_Temperature       => 43,
   Registered_Instruction    => 44,
   Reserved                  => 45,
   Moving                    => 46,
   Lock                      => 47,
   Punch_L                   => 48,
   Punch_H                   => 49);

const my %baudrateValues =>
  (1000000 => 0x01,
   500000  => 0x03,
   400000  => 0x04,
   250000  => 0x07,
   200000  => 0x09,
   115200  => 0x10,
   57600   => 0x22,
   19200   => 0x67,
   9600    => 0xCF);

# a received packet is deemed complete if no data was received in this much time
const my $timeDelimiter_s => 0.1;

# motor range in command coordinates and in degrees
const my $motorRange_coords => 0x400;
const my $motorRange_deg    => 300;

=head1 CONSTRUCTOR

=head2 new(PIPE, MOTORADDRESS)

Creates a new object to talk to a Dynamixel motor. The file handle has to be opened and set-up
prior to constructing the object.

=cut
sub new
{
  my ($classname, $pipe, $motoraddr) = @_;

  my $this = {pipe => $pipe,
              addr => $motoraddr};
  bless($this, $classname);

  return $this;
}

# Constructs a binary dynamixel packet with a given command
sub _makeInstructionPacket
{
  my ($addr, $instruction, $parameters) = @_;
  my $body = pack( 'C3C' . @$parameters,
                   $addr, 2 + @$parameters, $instruction,
                   @$parameters );

  my $checksum = ( ~sum(unpack('C*', $body)) & 0xFF );
  return pack('CC', 0xFF, 0xFF) . $body . chr $checksum;
}

# parses a given binary string as a dynamixel status packet
sub _parseStatusPacket
{
  my $str = shift;

  my ($key) = unpack('n', substr($str, 0, 2, '')) or return;

  return if($key != 0xFFFF);

  my ($addr, $length, $error) = unpack('C3', substr($str, 0, 3, '')) or return;
  my $numParameters = $length - 2;
  return if($numParameters < 0);

  my @parameters = ();
  my $sumParameters = 0;
  if($numParameters)
  {
    @parameters = unpack("C$numParameters", substr($str, 0, $numParameters, '')) or return;
    $sumParameters = sum(@parameters);
  }
  my $checksum = unpack('C1', substr($str, 0, 1, '')) // return;
  my $checksumShouldbe = ~($addr + $length + $error + $sumParameters) & 0xFF;
  return if($checksum != $checksumShouldbe);

  return {from   => $addr,
          error  => $error,
          params => \@parameters};
}

=head1 METHODS

=head2 pingMotor( )

Sends a ping. STATUS reply is returned

=cut

sub pingMotor
{
  my $this = shift;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};
  print $pipe _makeInstructionPacket($addr, $instructions{PING}, []);
  return pullMotorReply($pipe);
}

=head2 writeMotor(startingAddress, data)

Sends a command to the motor. STATUS reply is returned.

=cut

sub writeMotor
{
  my ($this, $where, $what) = @_;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};
  print $pipe _makeInstructionPacket($addr, $instructions{WRITE_DATA}, [$where, @$what]);
  return pullMotorReply($pipe);
}

=head2 readMotor(startingAddress, howManyBytes)

Reads data from the motor. STATUS reply is returned.

=cut

sub readMotor
{
  my ($this, $where, $howmany) = @_;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};
  print $pipe _makeInstructionPacket($addr, $instructions{READ_DATA}, [$where, $howmany]);
  return pullMotorReply($pipe);
}

=head2 writeMotor_queue(startingAddress, data)

Queues a particular command to the motor and returns the received reply. Does
not actually execute the command until triggered with triggerMotorQueue( )

=cut

sub writeMotor_queue
{
  my ($this, $where, $what) = @_;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};
  print $pipe _makeInstructionPacket($addr, $instructions{REG_WRITE}, [$where, @$what]);
  return pullMotorReply($pipe);
}

=head2 triggerMotorQueue( )

Sends a trigger for the queued commands. STATUS reply is returned.

=cut

sub triggerMotorQueue
{
  my $this = shift;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};
  print $pipe _makeInstructionPacket($addr, $instructions{ACTION}, []);
  return pullMotorReply($pipe);
}

=head2 resetMotor( )

Sends a motor reset. STATUS reply is returned.

=cut

sub resetMotor
{
  my $this = shift;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};
  print $pipe _makeInstructionPacket($addr, $instructions{RESET}, []);
  return pullMotorReply($pipe);
}

=head2 syncWriteMotor(startingAddress, data)

Sends a synced-write command to the motor. STATUS reply is returned.

=cut

sub syncWriteMotor
{
  my ($this, $writes, $where) = @_;
  my ($pipe, $addr) = @{$this}{qw(pipe addr)};

  my @parms = map { ($_->{addr}, @{$_->{what}}) } @$writes;
  my $lenchunk = scalar @{$writes->[0]{what}};
  @parms = ($where, $lenchunk, @parms);

  if( ($lenchunk + 1) * @$writes + 2 != @parms )
  {
    die "syncWriteMotor: size mismatch!";
  }

  print $pipe _makeInstructionPacket($BROADCAST_ADDR, $instructions{SYNC_WRITE}, \@parms);
  return pullMotorReply($pipe);
}

=head2 pullMotorReply( )

Performs a blocking read on the input pipe, and returns a parsed packet when
it is received.

=cut

sub pullMotorReply
{
  my $pipe = shift;

  # read data until there's a lull of $timeDelimiter_s seconds
  my $packet = '';
  select(undef, undef, undef, $timeDelimiter_s); # sleep for a bit to wait for data
  while(1)
  {
    my $rin = '';
    vec($rin,fileno($pipe),1) = 1;
    my ($nfound, $timeleft) = select($rin, undef, undef, $timeDelimiter_s);
    last if($nfound == 0);

    my $bytes;
    sysread($pipe, $bytes, $nfound);
    $packet .= $bytes;
  }

  return _parseStatusPacket($packet);
}


=head2 moveMotorTo_deg(position_degrees)

Convenience function that uses the lower-level routines to move a motor to a
particular position

=cut

sub moveMotorTo_deg
{
  my $this         = shift;
  my $position_deg = shift;

  my $position = int( 0.5 + ($position_deg * $motorRange_coords/$motorRange_deg + 0x1ff) );
  $position    = 0                    if $position <  0;
  $position    = $motorRange_coords-1 if $position >= $motorRange_coords;
  return $this->writeMotor($addresses{Goal_Position_L}, [unpack('C2', pack('v', $position))] );
}

1;


__END__


=head1 BUGS

There are several aspects of this module that are unideal and may change in the
future. The current implementation ties an instance of a Device::Dynamixel
object to a particular motor, NOT to a string of motors. This means that talking
to multiple motors requires multiple instances. This imposes a requirement that
only one motor can be controlled at any given time. Further, multi-motor
commands like syncWriteMotor( ) become essentially useless.

Another major issue is the baud rate of the serial communication. The motors
default to 1M baud. This is unsupported by the stock POSIX module in perl5, so
the serial port must be configured external to this module.

=head1 REPOSITORY

L<https://github.com/dkogan/dynamixel>

=head1 AUTHOR

Dima Kogan, C<< <dkogan at cds.caltech.edu> >>

=head1 LICENSE AND COPYRIGHT

Copyright 2011 Dima Kogan.

This program is free software; you can redistribute it and/or modify it
under the terms of either: the GNU General Public License as published
by the Free Software Foundation; or the Artistic License.

See http://dev.perl.org/licenses/ for more information.


=cut

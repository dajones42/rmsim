
# RMSim Startup File

RMSim startup files are ascii text files that contain line oriented
commands.  Lines are divided into tokens separated by a delimiter.  The
first token is a command name and the meaning of the remaining tokens
depends on the command.  The default delimiter is space or tab.  Any
text on a line after a # is a comment and is ignored.  Double quotes can
be used to create tokens that contain a delimiter.

Some commands use multiple lines.  Multiline commands end with a line that
contains an end command.  Most multiline commands use named subcommands that
depend on the command.  Other multiline commands use a table of values.

Tokens that are expected to be numbers are parsed as python expressions
if they contain non-numeric characters.

Some commmands use 3D coordinates to identify a location in the scene.
The coordinates are easting, northing and altitude relative to center.
The easiest way to find the coordinates of a point is to run the simulation
and click on the scene.  The coordinates will be written to stderr.

RMSim use a right handed coordinate system with x forward, y sideways and z up.

UTF8 characters used in startup files will be passed as is to the OS or OSG.
So they may or may not work.  Non-ascii characters in MSTS files are converted
into UTF8 internally.

## Top Level Commands

- delimiters *characters*: Changes the delimiter to the characters in the
    second token.
- include *file*: Reads commands from the file specified by the second token.
- python *code*: Interprets the remainder of the line as a python
    statement.
- if *options*: Processes the following commands if any of the remaining
    tokens was specified on the command line.
- elif *options*: Terminates the previous if and starts another.
- else: Terminates a previous if and processes the following commands if the
    previous if test failed.
- endif: Terminates previous if or else processing.
- mstsroute *mstsdir* *routeid*: Starts multiline processing of MSTS route.
- railcar *name*: Starts multiline definition of a railroad car.
- wag *file*: Starts multiline definition of a railroad car initialized
    from an MSTS wag or eng file.
- train *name*: Starts multiline definition of a train.
- signal *name*: Starts multiline definition of an interlocking signal.
- interlocking *numlevers*: Starts multiline definition of an interlocking.
- timetable *name*: Starts multiline definition of a timetable.
- useroscallsign *call*: Identifies the user's timetable station.
- connect *name1* *name2*: Connects tracks named by the next two tokens.
- model2d *name*: Starts multiline definition of a 2D shape.
- model3d *name*: Starts multiline definition of a 3D shape.
- texture *name*: Starts multiline definition of texture image.
- scenery *name* *easting* *northing* *altitude* *angle*:
    Adds a 3D model to the scene.
- interlockingmodel *name*: Identifies a 3D model to be used for
    interlocking levers.
- switchstand *trackname* *easting* *northing* *altitude* *modelname*:
    Adds a switch stand 3D model to a track switch.
- throwswitch *trackname* *easting* *northing* *altitude*: Changes the
    position of a track switch.
- starttime *hour* *minute*: Specifies the simulation start time (24 hours).
- randomize: Sets a random number seed based on the current time.
- morse: Start a multiline definition of morse code sound effects.
- person *easting* *northing* *altitude*: Specifies the coordinates of a
    person in the scene.
- personpath: Starts a multiline definition of a path a person can move along.
- location *easting* *northing* *altitude* *locname* *trackname*:
    Defines a named track location.
- align *locname1* *locname2*: Aligns all switches between two named
    track locations.
- track *name*: Starts multiline definition of railroad track.
- trackshape *name*: Starts multiline definition of a track cross section.
- splittrack *name* *newname* *easting* *northing* *altitude* *angle*:
    Divides the track named by the first token into two pieces
    using a line defined by the other tokens.
- water: Starts multiline definition of navigable water.
- ship *name*: Starts multiline definition of a ship named by next token.
- floatbridge *name*: Starts multiline definition of a float bridge.
- rope *ship1* *x1* *y1* *ship2* *x2* *y2*: Adds a rope between two cleats
    identified by the next six tokens.

## MSTS Route Subcommands

- filename *fileprefix*: filename prefix if different from route directory name.
- center *tilex* *tilez* *latitude* *longitude*: overrides calculated center
    of route.
- ignoreHiddenTerrain: causes hidden terrain flags to be ignored.
- signalswitchstands: enables signals that act as switch stands.
- createsignals: turns on limited signal support.
- trackoverride *shapename* *altshapename*: Overrides a track shape with an
    alternate model.
- drawwater: enables rendering of water.
- waterleveldelta *delta*: raises rendered water by the specified value.
- activity *actfile* *flags*: populate loose consists from the specified
    activivty.  Populates player train if flags is 1.
- switchstand *sideoffset* *altoffset* *pointoffset* *model*:  Adds an
    animated switch stand model to all junctions in the route.

## RailCar Definition Subcommands

- axles *number*: Sets the number of axles.
- mass *emptymass* *loadedmass*: Sets the car mass (Mg).
- rmass *mass*: Sets rotating mass equivalent (Mg).
- drag0a *value*: Sets Davis equation axle multiplier. 
- drag0b *value*: Sets Davis equation mass multiplier. 
- drag1 *value*: Sets Davis equation speed multiplier. 
- drag2 *value*: Sets Davis equation speed squared multiplier. 
- drag2a *value*: Sets Davis equation speed squared multiplier.
- area *value*: Sets cross section area (m^2).
- length *value*: Sets the car length between couplers (m).
- offset *value*: Sets car model offset from center (m).
- width *value*: Sets car width (m).
- maxbforce *value*: Sets maximum brake force (kn).
- steamengine: Starts multiline definition of steam engine.
- dieselengine: Starts multiline definition of diesel engine.
- electricengine: Starts multiline definition of electric engine.
- smoke *x* *y* *z* *xdir* *ydir* *zdir* *size* *minrate* *maxrate*
    *minspeed* *maxspeed*: Adds smoke effect to model.
- copy *name*: Copy data from another railcar definition.
- mstsshape *file*: Sets MSTS shape file to use for car.
- print *number*: Prints model details for specified part.
- printparts: Prints part heirarchy.
- remove *number* *name*: Disables rendering of model parts.
- inside *xoffset* *yoffset* *zoffset* *heading* *vangle*:  Adds an
    inside view location.
- sound *file*: Adds a sound wav file for diesel or sound sms file for steam.
- brakevalue *name*: Sets the type of brake valve (currently recognized values
    K, AB, H6, L or AMM).
- headlight *x* *y* *z* *radius* *unit* *color*: Adds a headlight.

### Steam Engine Definition Subcommands

- cyldiameter *value*: cylinder diameter (in).
- cylstroke *value*: cylinder stroke (in).
- mainrodlength *value*: main rod length (in).
- wheeldiameter *value*: wheel diameter (in).
- numcylinders *number*: number of cylinders.
- maxpressure *value*: max boiler pressure (psig).
- clearancevolume *value*.
- boilervolume *value*.
- waterfraction *value*: normal fraction of water in boiler.
- shovelmass *value*: mass of shovel full of coal for hand firing.
- gratearea *value*.
- firingrateinc *value*.
- blowerrate *value*.
- auxsteamusage *value*.
- safetyusage *value*.
- safetydrop *value*.
- cylpressuredrop: Starts multiline date table.
- backpressure: Starts multiline date table.
- release: Starts multiline date table.
- evaprate: Starts multiline date table.
- burnrate: Starts multiline date table.
- burnfactor: Starts multiline date table.
- evapfactor: Starts multiline date table.

### Diesel Engine Definition Subcommands

- maxforce *value*: Sets maximum tractive force (kn).
- maxcforce *value*: Sets maximum continuous tractive force (kn).
- maxpower *value*: Sets maximum power (kw).
- notches *number*: Sets number of throttle notches.

### Electric Engine Definition Subcommands

- maxforce *value*: Sets maximum tractive force (kn).
- maxcforce *value*: Sets maximum continuous tractive force (kn).
- minaccelforce *value*: Sets minimum tractive force used for automatic
    acceleration feature (kn).
- maxpower *value*: Sets maximum power (kw).
- notches *number*: Sets number of throttle notches.
- topspeed *value*: Sets top speed (mps).  Used to calculate maxcforce if
    not provided (maxcforce=maxpower/(.6*topspeed)).
- efficiency *value*: Sets motor and transmission effeciency.
- powerfactor *value*: Sets power factor for AC motors..
- pantographs *forward* *reverse*: identifies directional pantographs.
- step *notch* *voltageRatio* *resistanceMult*: Adds an automatic acceleration
    step.  Voltage ratio should be between 0 and 1 (e.g. 1 for motors in
    parallel and .5 for two motors in series).  Resistance mult should be
    one or more, 1 for no extra resistance and 2 for extra resistance equal
    to the motor resistance.

## Train Definition Subcommands

- brakes *maxeqres* *initeqres* *initaux* *initcyl* *brakevalve*:
   Sets brake pipe pressures (psig) and default brake valve.
- car *name* *loadfraction*: add a defined car to train.
- wag *dir* *file*: adds a car defined by an MSTS wag or eng file to train.
- waybill *name* *red* *green* *blue* *probability*:  Adds a waybill to the
    previous car.
- xyz *easting* *northing* *altitude*: Sets the location for the center of
    the train.
- txyz *name* *easting* *northing* *altitude*: Sets the location for the
    center of the train on named track.
- loc *name*: Sets the train location to a named location.
- reverse: Reverses the train direction on track.
- speed *value*: Sets target speed for AI trains (mps).
- accelmult *mult*: Sets acceleration multiplier for AI trains.
- decelmult *mult*: Sets deceleration multiplier for AI trains.
- move *dist*: Move the train the specified distance down the track.
- pick *min* *numeng* *nnumcab* *max*: Randomly selects cars.

## Signal Definition Subcommands

- track *easting* *northing* *altitude* *reverse*: Associates signal with
    track location and direction.
- distant: Marks signal as distant.

Multiple track locations can be associated with a single signal.

## Interlocking Definition Subcommands

- lever *number* *red* *green* *blue*: adds a locking or spare lever. 
- siglever *number* *red* *green* *blue* *name*: adds a signal lever. 
- swlever *number* *red* *green* *blue* *easting* *northing* *altitude*
    *[reverse]*: Associates a switch with a lever. 
- locking *{states}*: Adds locking between lever states.
- when *{states}* locking *{states}*: Adds conditional locking.
- image *file*: Adds drawing of interlocking that appears in web server
    display.

Lever states are tokens that contains a lever number followed by N, R or B.

Multiple switches can be associated with a single lever.

## Timetable Definition Subcommands

## Ship Definition Subcommands

- draft *value*: depth of ship (m).
- lwl *value*: length of water line (m).
- bwl *value*: beam at water line (m).
- mass *m* *sx* *sy* *sz*: Sets ship mass (Mg) and moments of inertia.
- propeller *value*: propeller diameter (m).
- power *value*: shaft power (kw).
- position *easting* *northing*: location of ship.
- heading *value*: initial compass heading of ship (degrees).
- model3d *name*: name of ship 3D model.
- modeloffset *value*: vertical offset of model.
- boundary *name*: name of 2d model for ship boundary used in collision
    detection.
- track *name*: name of track on ship.
- cleat *x* *y* *z* *standingZ*: Adds a cleat to ship is specified location.
- calcdrag *thrustfactor* *speed* *power* *diameter*: Calculates
    ship drag values to be equal to the propeller thrust given the specified
    conditions.


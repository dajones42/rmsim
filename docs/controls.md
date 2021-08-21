
# RMSim Controls

RMSim uses a combination of mouse clicks, single character key strokes and
multiple character controls.  Many keys have multiple meaning depending on the
current mode.

OSG converts keyboard scan codes into ascii character codes.  Most commands
recognize the ascii character codes.

## View Controls

Displays in RMSim are relative to the location of a person in the scene.
There are 3 different cameras: one looks from the person's location,
one looks at the person's location and the third looks down on the
person's location.  The person's location can be changed by clicking on the
scene.  If the clicked on location is nearby, the person will move to the
clicked location at walking speed.  The 'j' key can be used to cause the
person to jump immediately to the clicked location.  If the clicked on
location is a rail car, the person will move to the nearest
corner of the rail car and will follow the movement of the car.

- 1: Select look from view.
- 2: Select look at view.
- 3: Select look down on view.
- 4: Move person to the next inside rail car view (if any).
- 5: Select person number 1.
- 6: Select person number 2.
- 7: Select person number 3.
- 8: Select person number 4.
- 9: Select person number 5.
- 0: Toggle display of person model (if defined).
- Left: Rotate camera.
- Right: Rotate camera.
- Up: Zoom camera in.
- Down: Zoom camera out.
- PageUp: Rotate camera vertically.
- PageDown: Rotate camera vertically.
- F5: Increment HUD state.
- F6: Toggle track labels.
- F7: Toggle car waybill display.
- (: Move person left along train or predefined path.
- ): Move person right along train or predefined path.
- Return: Stop person's movement.
- Space: Set camera defaults.
- p: Toggle drawing of track path.
- P: Toggle drawing of track profile.

## Train Controls

To take control of a train, position a person on one of the train's cars and
press the 'c' key.  The train brake is an automatic air brake and the train
brake control has three positions: release, lap and service.

- w: Increase direction control.
- s: Decrease direction control.
- a: Decrease throttle.
- d: Increase throttle.
- Semicolon: Decrease train brake.
- Quote: Increase train brake.
- [: Decrease engine brake.
- ]: Increase engine brake.
- /: Bail off engine brake.
- {: Decrease hand brake on current car.
- }: Increase hand brake on current car.
- Colon: Decrease retainer on current car.
- DoubleQuote: Increase retainer on current car.
- c: take control of train.
- r: release control of train.
- C: connect air brake hoses.
- g: throw switch nearest to the current person (person must not be riding on
    a car).
- u: uncouple cars nearest to the current person (person must not be riding on
    a car).
- h: Turn headlight on.
- H: Turn headlight off.

### Conductor train controls

- >: Move forward.
- <: Move backward.
- ^: Increase speed.
- Comma: Decrease speed.
- Period: Stop.

### Firing controls

- f: Turn on manual firing.
- F: Turn on automatic firing.
- i: Toggle injector 1.
- k: Increase injector 1.
- K: Decrease injector 1.
- o: Toggle injector 2.
- l: Increase injector 2.
- L: Decrease injector 2.
- n: Increase blower.
- N: Decrease blower.
- m: Increase damper.
- M: Decrease damper.
- r: Increase firing rate or add one shovel full to fire.
- R: Decrease firing rate.

## Ship Controls

To take control of a ship, position a person on the ship and press the 'c' key.

- a: Decrease throttle.
- d: Increase throttle.
- s: set throttle to zero.
- j: move rudder 5 degrees left.
- J: move rudder 1 degree left.
- k: center rudder.
- l: move rudder 5 degrees right.
- L: move rudder 1 degree right.
- C: connect float bridge.
- u: remove ropes.
- e: ease rope near current person.
- t: tighten rope near current person.
- h: hold rope near current person.

To add a rope move the current person to one cleat and then control click on
another ship's cleat or a ground location.

## Time Controls

- x: Double time multiplier.
- z: Half time multiplier.
- W: Start automatic fast time.

## Interlocking Controls

When an interlocking is defined, levers will be displayed in the HUD.
Clicking on a HUD lever will toggle the lever's state.  If an interlocking
levers model is present, clicking on a lever will toggle its state.
Clicking on a train number in the HUD will record the train's passing on
the train sheet.

When an interlocking is defined, right click or shift left click will move
the current person.

## Multi-character controls

Multi-character controls start with '!' and end with enter/return.  Backspace
and delete can be used for editing.  Characters between '!' and enter are split
into '|' delimited tokens.  The first token is the command name and use of the
other tokens depends on the command.

- print ts: Print time sheet on stderr.
- print tsh: Print horizontal time sheet on stderr.
- print tt: Print timetable on stderr.
- print tth: Print horizontal timetable on stderr.
- ts: Display time sheet on HUD.
- meet|train1|train2|station: Creates a meet train order between the named trains.
- block for|train: Request manual block for the named train.
- forces: Print train car forces on stderr.
- os|train: Record the named train's passing on the train sheet.
- save|filename: Save the simulation state to the name startup file.
- exit: Exit the simulation.
- auto: Start automatic switcher on selected train.

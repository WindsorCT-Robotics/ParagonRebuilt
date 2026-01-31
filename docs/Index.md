# Rebuilt Robot Code Documentation

## Table of Contents

* Subsystems
  * [Drive](Subsystems/Drive.md)
  * [Shooter](Subsystems/Shooter.md)
  * [Indexer](Subsystems/Indexer.md)
  * [Vision](Subsystems/Vision.md)
  * [LED](Subsystems/LED.md)
* [Autonomous](Autonomous.md)
* [Sim](Sim.md)

## Dependencies

WPILib comes with all necessary dependencies. The following VSCode extensions are also standard-issue:

* Live Share
* GitLens
* Extension Pack for Java
* GitHub Pull Requests

## Basic Program Structure

All robot code will be in the package `frc.robot`. Subsystems will be located in the `frc.robot.subsystems` package. Stand-alone commands will be located in the `frc.robot.commands` package.

Hardware configuration and programming will be done in the `frc.robot.hardware` package. Hardware configuration should be kept separate from subsystem code so that subsystem code can focus on the behaviors of the robot and hardware code can evolve or be replaced independently of a given subsystem.

Subsystems will take dependencies on interfaces that describe the behaviors they require to operate. These interfaces will be stored in the `frc.robot.interfaces` package. Hardware classes must implement these interfaces.

Auto-generated code must be stored under the `frc.robot.generated` package. This not only keeps auto-generated code out of our way, but also ensures that the static analysis tool we use -- SonarQube -- does not attempt to analyze the code.

Finally, Sim code should be in a package underneath `frc.robot.sim`. The sim package will likely have many sub-packages that mirrors the above package structure (e.g., `frc.robot.sim.subsytems`).

## Naming Conventions

### Type and Field Names

Class names should describe a noun, verb, or action. The name should not contain the package name (e.g., Elevator, NOT ElevatorSubsystem).

Commands will be verbs, Subsystem will be nouns, and booleans will be framed as true/false questions (isOn, shouldRun, etc).

### Encapsulation

Classes should keep all of their state `private`. Fields should be delcared `final`, and only not final when necessary.

* Only `Command`s and `Trigger`s should be public
    * Where appropriate, Triggers should be debounced. Almost all sensor and button triggers should be debounced.
* Other data may be made `public` rarely if required.
    * This should be an absolute last resort. Data should be encapsulated as much as possible.
* Public data should be made `public` through `Supplier`s.
    * Suppliers are functions that take in no parameters and return a value. They are essentially getters.
* `Consumer`s should be used for Smart Dashboard setup and rarely anything else.
    * Consumers are functions that take in a parameter and return no value. They are essentially setters.
    * Consumers that call functions that generate Commands should also make sure to schedule them by calling the `schedule` method on the command.

### Subsystems

Subsystem class names should be nouns that match up to the physical part of the robot it controls. Subsystems may consist of multiple motors or sensors, just like a physical system in the robot might use multiple motors or sensors to do its job.

Subsystem constructors should receive interfaces representing the motors and sensors it requires.

Subsystems will expose functionality as Commands and state as Triggers.

### Commands

Stand-alone commands will exist when the command cannot logically exist in a single subsystem.

### Interfaces

Interfaces names should always start with a capital `I` and describe the capability the interface requires. Interfaces are conceptually contracts and names should reflect that (e.g., IDistanceMotor for a motor with an encoder configured to convert distances to rotations).

Interfaces can build on each other; functions should not be repeated among interfaces and shared interface functions should be extracted into their own interfaces (e.g., all motor interfaces extending `IMotor`, which has functions that are common among all motors)

### Auto-Generated Code

Auto-Generated Code is considered SOUP: Software of Unknown Provenance. It should not be modified, and should live in a `generated` package. Our code should extend it or otherwise use it without changing it.

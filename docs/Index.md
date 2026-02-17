# Rebuilt Robot Code Documentation

## Table of Contents

* [Robot Container](RobotContainer.md)
* Subsystems
  * [Drive](Subsystems/Drive.md)
  * [Shooter](Subsystems/Shooter.md)
  * [Indexer](Subsystems/Spindexer.md)
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

## Basic Architecture Overview

### Basic Program Structure

All robot code will be in the package `frc.robot`. Subsystems will be located in the `frc.robot.subsystems` package. Stand-alone commands will be located in the `frc.robot.commands` package.

Hardware configuration and programming will be done in the `frc.robot.hardware` package. Hardware configuration should be kept separate from subsystem code so that subsystem code can focus on the behaviors of the robot and hardware code can evolve or be replaced independently of a given subsystem.

Subsystems will take dependencies on interfaces that describe the behaviors they require to operate. These interfaces will be stored in the `frc.robot.interfaces` package. Hardware classes must implement these interfaces.

Auto-generated code must be stored under the `frc.robot.generated` package. This not only keeps auto-generated code out of our way, but also ensures that the static analysis tool we use -- SonarQube -- does not attempt to analyze the code.

Finally, Sim code should be in a package underneath `frc.robot.sim`. The sim package will likely have many sub-packages that mirrors the above package structure (e.g., `frc.robot.sim.subsytems`).

### Auto-Generated Code

Auto-Generated Code is considered SOUP: Software of Unknown Provenance. It should not be modified, and should live in a `generated` package. Our code should extend it or otherwise use it without changing it.

### Units

Primitives should not be used where Units can be used instead. For example, every distance is a double, but not every double is a distance. Distance should be used instead.

Creating new types is preferable to using primitives in most cases, if for no other reason than to ensure values don't get accidentally swapped around.

Values in percentages will be represented by variables of the `Dimensionless` type.

### `null` and the Absence of Value

In Java, any object is allowed to be assigned a `null` value. This presents problems, as accessing an object with a `null` value will cause an application to immediately throw an `exception`.

To prevent this, `null` should be avoided at all costs. If a public API's documentation doesn't explicitly state that it requires or returns `null` values, it should not be used.

In our code, we shall *always* use the `Optional` type to express values that might not return a value.

### Errors

In Java, methods may throw exceptions for error handling. Errors that are "exceptional" -- things that should *never* happen during the normal execution of a program -- or programming errors are to be handled by throwing exceptions. Exceptions should be used sparingly, as they will essentially crash the program if not handled correctly.

Exceptions should *never* be used for control flow. If a failure is a known possibility and/or an expected outcome of a function, the `Result` type must be used instead. The `error` type parameter of the `Result` type should explain why the operation failed.

When possible, errors should be left to deal with by the calling function. Subsystems should not concern themselves with failures not caused by private methods. The most effective way to deal with this is the `Result` type. `exceptions` can do the same thing, but throwing an exception means that something has gone terribly wrong.

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
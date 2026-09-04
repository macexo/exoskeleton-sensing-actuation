# Contributing to McMaster Exoskeleton — Sensing & Actuation

Thanks for contributing! This repository holds the embedded code for the sensing and
actuation subsystem: sensor drivers, motor control, and the firmware that ties them
together on the exoskeleton. See the [README](README.md) for what the subsystem does,
the hardware it targets, and the current state of each project.

Because this code drives real motors on a device a person wears, please read
[Hardware & Safety Guidelines](#hardware--safety-guidelines) before you flash anything.

## Table of Contents

- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)
- [Development Workflow](#development-workflow)
- [Coding Standards](#coding-standards)
- [Commit Guidelines](#commit-guidelines)
- [Pull Request Process](#pull-request-process)
- [Testing](#testing)
- [Documentation](#documentation)
- [Hardware & Safety Guidelines](#hardware--safety-guidelines)

## Getting Started

### Prerequisites

| Tool | Version | Used for |
| --- | --- | --- |
| [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) | 1.14+ | Building and flashing STM32 firmware |
| Python | 3.9+ | Host-side test and logging scripts |
| Git | any recent | Version control |

Target hardware: **STM32 Nucleo-F446RE**. Motors communicate over **CAN**; sensors over
**I2C**, **SPI**, or **UART** depending on the part.

### Setting Up

1. Clone the repository:

    ```bash
    git clone https://github.com/macexo/exoskeleton-sensing-actuation.git
    cd exoskeleton-sensing-actuation
    ```

2. Open the project you want to work on in STM32CubeIDE:

    **File → Open Projects from File System…** → select a project folder under
    `testing/stm32/` or `src/stm32/` (the one containing the `.project` and `.ioc`
    files). Each project is standalone — do not import the repository root.

3. Install Python dependencies for the host-side scripts:

    ```bash
    pip install -r requirements.txt
    ```

4. Verify your setup before writing any code:

    - Build the project (**Project → Build All**) and confirm it compiles with no warnings.
    - Flash to the Nucleo and confirm the board enumerates over ST-Link.
    - Run the project's test script to confirm you can talk to the board over UART.

If any of these fail, ask in the team Discord channel before continuing.

## Repository Structure

The layout is documented in the [README](README.md#repository-structure) and matches
`exoskeleton-embedded`: reusable hardware APIs under `apis/<domain>/`, working control
loops under `src/`, and standalone bring-up projects under `testing/`.

Put new code in the right one:

- Writing something two projects will use? `apis/<domain>/`, split `Inc/` and `Src/`.
- Proving out a new sensor, motor, or API on the bench? `testing/stm32/<project>/`.
- Promoting a proven bring-up into a real control loop? `src/stm32/<project>/`.

Three things are expected of anything you add:

- **Document it** — an `<domain>-api.md` for an API, a `README.md` for a project, covering
  what it does, how to build it, and how to test it.
- **Add it to the project table** in the [README](README.md#projects) — add the row when
  you start, so nobody duplicates your work, and update the status when it lands.
- **Don't duplicate an API into a project.** If you need to change a shared API to make
  your project work, change it in `apis/` and say so in the PR, rather than copying the
  files in and editing the copy.

## Development Workflow

### Branch Naming

Branch off `main`. Use a short, descriptive name in the form `type/description`:

- `feature/description` — New features
- `bugfix/description` — Bug fixes
- `refactor/description` — Code refactoring
- `docs/description` — Documentation updates
- `test/description` — Test additions/updates

### Creating a Branch

```bash
git checkout main
git pull
git checkout -b feature/your-feature-name
```

Keep branches focused and short-lived. One logical change per branch makes review far
faster than a branch that touches five unrelated things.

### What Not to Commit

Build output and IDE state are ignored by `.gitignore` — keep it that way. Never commit
`Debug/`, `Release/`, `*.elf`, `*.bin`, `*.hex`, `*.map`, or `.metadata/`.

**Do** commit the `.ioc` file whenever you change the CubeMX configuration, along with
the regenerated code, so everyone's build matches the hardware config.

## Coding Standards

Firmware in this repository is **C (C99)** built against the STM32 HAL. Host-side tooling
is Python.

### Naming Conventions

| Element | Convention | Example |
| --- | --- | --- |
| Files | `snake_case` | `lsm6dso32_port_spi.c`, `lsm6dso32_port.h` |
| Functions | `snake_case`, prefixed with the module name | `lsm6dso32_spi_ctx()` |
| Variables | `snake_case` | `sensor_id`, `accel_raw` |
| Types (`struct`/`enum` typedefs) | `snake_case` with `_t` suffix | `lsm6dso32_spi_bus_t` |
| Macros and constants | `UPPER_SNAKE_CASE` | `LSM6DSO32_SPI_TIMEOUT_MS` |
| Static (file-local) helpers | `static`, `snake_case` | `static void reset_bus(void)` |

### Code Formatting

- 4 spaces per indentation level. No tabs.
- Maximum line length: 100 characters.
- Braces on the same line as the statement (`if (cond) {`), always braced even for
  single-statement bodies.
- One declaration per line.
- Do **not** reformat CubeMX-generated code — it creates noisy diffs and gets overwritten.

### Writing Code in CubeMX Projects

STM32CubeIDE regenerates `Core/` from the `.ioc` file, **silently deleting anything
outside the user-code markers**. Always write inside them:

```c
/* USER CODE BEGIN 2 */
static lsm6dso32_spi_bus_t imu_bus = { &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin };
stmdev_ctx_t imu = lsm6dso32_spi_ctx(&imu_bus);
/* USER CODE END 2 */
```

Application logic that isn't tied to peripheral init belongs in its own `.c`/`.h` pair
rather than in `main.c`.

### Vendor Code

Where a chip vendor ships a good platform-independent driver, use it rather than writing
a register map from scratch — ST's `lsm6dso32_reg.c/.h` is the model here.

Vendor sources are **vendored unmodified**. Don't reformat them, don't fix their style,
and don't patch them in place: write a thin port layer that implements the bus functions
the vendor driver expects (`read_reg`, `write_reg`, `mdelay`) and keep it in a separate
file named for its bus, such as `lsm6dso32_port_spi.c`. Note the upstream URL and version
in the driver's `README.md` so the next person can diff against it or pull a fix.

If a vendor driver genuinely needs a change, say so in the PR and add a comment at the
patch site explaining why — an undocumented edit to vendored code silently disappears the
next time someone re-vendors it.

### Error Handling

Return an explicit status code rather than `void` or a bare `bool` for anything that can
fail — a status enum tells the caller *why* something failed.

```c
typedef enum {
    LSM6DSO32_OK       =  0,  // Success
    LSM6DSO32_ERR_BUS  = -1,  // Bus transaction failed
    LSM6DSO32_ERR_NULL = -2,  // NULL pointer argument
    LSM6DSO32_ERR_ID   = -3,  // WHO_AM_I mismatch
} lsm6dso32_status_t;
```

Check every pointer argument for `NULL`, and check the return value of every HAL call.

### Documentation Comments

Use Doxygen-style comments on every public function, with `@brief`, `@param`, and
`@return` where applicable. Keep the documentation for a type and its functions in the
header that declares them, and update comments in the same commit as the code they
describe.

#### Example

```c
/**
 * @brief  Read sensor data from the specified sensor.
 * @param  sensor_id  Sensor identifier (0 to MAX_SENSORS-1).
 * @param  data       Pointer to the data buffer to fill.
 * @retval SENSOR_OK on success, a negative sensor_status_t code otherwise.
 */
sensor_status_t sensor_read(uint8_t sensor_id, sensor_data_t *data);
```

## Commit Guidelines

### Commit Message Format

Follow the [Conventional Commits](https://www.conventionalcommits.org/) format:

```
<type>(<scope>): <subject>

<body>

<footer>
```

Write the subject in the imperative mood ("add", not "added"), keep it under 72
characters, and use the body to explain *why* the change was made.

**Types**:

- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Test additions/updates
- `chore`: Build process or auxiliary tool changes

**Scopes**: the driver, project, or subsystem touched — for example `motor`, `imu`,
`can`, `pi`.

**Examples**:

```
feat(motors): add position control mode

Implement PID position control for motor drivers with configurable
gains and limits.

Closes #123
```

```
fix(canbus): handle extended frame format correctly

Fix bug where extended CAN frames were not being parsed correctly,
causing message loss.
```

## Pull Request Process

1. **Rebase on `main`** so your branch merges cleanly.
2. **Update documentation** for any behaviour you changed.
3. **Update tests** for new or changed functionality.
4. **Check the build** — it must compile with no new warnings.
5. **Test on hardware** if the change touches firmware or a driver.
6. **Open the PR** with a descriptive title, a link to the related issue, a summary of
   what changed and why, and instructions for how a reviewer can test it.
7. **Request a review.** PRs need at least one approval before merging; don't merge your
   own PR without one.

### PR Checklist

- [ ] Code follows the style guidelines above
- [ ] Self-review completed
- [ ] Comments added for non-obvious logic
- [ ] Documentation updated
- [ ] No new build warnings
- [ ] Build artifacts and IDE state are not committed
- [ ] Hardware tested (if applicable) — state the setup used
- [ ] Tests added/updated and passing

## Testing

### Bench Testing in `testing/`

Most testing here happens on hardware, in a standalone project under `testing/stm32/`
that exercises one peripheral or API at a time. Build the bring-up project first, prove
the part works in isolation, and only then wire it into a control loop under `src/`.

### Host-Side Unit Tests

Where an API's logic is separable from the bus — frame packing, fixed-point scaling,
feedback parsing — it can be tested on your laptop by mocking the bus functions. These
tests run fast and catch encoding bugs before a board is involved, which is worth doing
for anything that computes a value sent to a motor.

### Hardware Testing

Hardware behaviour is not covered by unit tests, so test on the target before you open a
PR. In the PR description, record:

- The board and revision used, and the sensors or motors connected.
- The firmware project and branch flashed.
- The script or procedure you ran, and the result.

### Coverage Expectations

Prioritise coverage where failures are expensive: torque and current limits, CAN message
encoding and decoding, state machine transitions, and any fault or shutdown path. These
are the paths that protect the person wearing the device.

## Documentation

### Code Documentation

- Document every public API in the header that declares it.
- Keep each project's `README.md` current with build and usage instructions.
- Document hardware requirements, wiring, and pin assignments alongside the firmware
  that depends on them.

### Inline Comments

- Explain *why*, not *what* — the code already says what it does.
- Cite the datasheet page or manual section for magic numbers, register addresses, and
  scaling factors.

## Hardware & Safety Guidelines

**Target platform**: STM32 Nucleo-F446RE (STM32F446RE, ARM Cortex-M4F @ 180 MHz)

**Buses**: CAN for motor control; I2C, SPI, and UART for sensors.

### Safety Rules for Actuation Testing

These are non-negotiable, because this subsystem moves joints under power:

1. **Never test motors alone.** Have a second person present with access to the power
   cut-off.
2. **Bench-test before body-test.** Any new control code runs on an unloaded, securely
   mounted motor before it goes anywhere near the exoskeleton frame or a person.
3. **Enforce limits in code.** Clamp torque, velocity, and position commands to the
   documented limits for the specific motor. Never send an unclamped value from a
   host script straight to a motor.
4. **Fail safe.** Every control loop needs a watchdog or timeout that disables the motor
   when commands stop arriving. Loss of communication must mean motor off, not last
   command held.
5. **Keep the e-stop reachable** and verify it works *before* enabling the motor.

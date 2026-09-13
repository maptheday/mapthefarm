# C++ headers and `extern`, built up from zero

*Assumes you know C#, and nothing about C++ headers yet. Every word is defined the moment before it's used.*

---

## What we're going to build

A tiny program about an **animal shelter**:

- there is **one** shelter,
- **dogs** and **cats** "arrive," and when they do they **check in** to that one shelter.

We'll write it in small pieces. Each time a new C++ idea shows up, we stop, name it, and compare it to how you'd do the same thing in C#.

---

## Word 1 — a *class* is a *blueprint* (you already know this from C#)

A **class** — C++ also spells it `struct`, treat them as the same thing here — is a **blueprint**: it describes what a thing *is* and what it can *do*. The class itself isn't a thing; it's the design.

An **object** is an actual thing built from that blueprint.

```
   blueprint (class)          object (a real one)
      "Shelter"        ──►     a real shelter, animals = 0
```

This is identical to C#, so nothing is new yet:

```csharp
class Shelter { public int Animals; }   // the blueprint
var s = new Shelter();                   // an object built from it
```

**Keep this: blueprint = class = the design. object = one real instance of it.**

---

## Word 2 — a *header*, and why C++ even has them

Here's the first real difference from C#.

In **C#**, you put `.cs` files in a project and the compiler reads them **all together** — every file can automatically see every class.

In **C++**, each code file is compiled **alone, in its own bubble.** `Dog`'s file cannot automatically see `Shelter`'s file. So how does the Dog code learn what a `Shelter` even is?

You put the `Shelter` blueprint in a **header file** — `Shelter.hpp` — and any file that needs it writes:

```cpp
#include "Shelter.hpp"
```

`#include` **literally pastes the entire text of that file in, right there,** before compiling. So:

> A **header** is a file of blueprints (and, soon, *promises*) that you **paste** into other files so they know what exists.

C# has no headers because its compiler already sees everything — as if every file were auto-included into every other. `#include` is the manual version of that.

---

## Piece 1 — the Shelter blueprint

```cpp
// Shelter.hpp
#pragma once                 // "if this file gets pasted twice, ignore the 2nd paste"

struct Shelter {             // just the blueprint
  int animals = 0;
  void checkIn() { animals++; }
};
```
```csharp
// C# equivalent
class Shelter { public int Animals = 0; public void CheckIn() => Animals++; }
```

This is *only* the blueprint — no shared object yet. Any file may `#include "Shelter.hpp"` to learn what a Shelter is. **Pasting a blueprint into many files is totally fine** — they're all just learning the same design.

---

## Piece 2 — a Dog that uses a shelter

A Dog needs to know what a Shelter is, so its file includes the header:

```cpp
// Dog.hpp
#pragma once
#include "Shelter.hpp"       // now this file knows the Shelter blueprint

struct Dog {
  void arrive() {
    shelter.checkIn();       // ...but WHICH shelter? that's the next section
  }
};
```
```csharp
// C# equivalent
class Dog { public void Arrive() => Registry.Shelter.CheckIn(); }
```

`shelter` here means "the *one* shared shelter object." We haven't created it yet — and creating it is the whole point of `extern`.

---

## The problem — we want ONE shelter, shared by everyone

We don't want each dog to build its own shelter. We want a single shelter object the whole program shares.

In **C#** that's a one-liner — declared and created in one place, and everyone just says `Registry.Shelter`:

```csharp
static class Registry {
    public static Shelter Shelter = new();   // one object; everyone uses Registry.Shelter
}
```

In **C++** it's trickier, and here's exactly why. Remember: each file compiles alone, and `#include` *pastes*. If we wrote the line that **creates** the shelter inside `Shelter.hpp`, then every file that pastes that header would create **its own** shelter. Three files that include it → three different shelters with the same name → the build fails at the very end with:

```
error: multiple definition of 'shelter'
```

To avoid that, C++ makes you split the shared object into **two separate lines**:

**1. The promise** (called a *declaration*) — goes in the header, safe to paste everywhere:

```cpp
extern Shelter shelter;   // "a Shelter object named `shelter` exists SOMEWHERE. I'm not making it."
```

`extern` means exactly *"this object is real and lives in some other file — here's its name and type so you can use it."* It creates nothing.

**2. The real object** (called a *definition*) — written once, in one file:

```cpp
Shelter shelter;          // actually builds the object, one time
```

Put line 1 in `Shelter.hpp` (so every file gets the promise when it `#include`s), and line 2 in a single owner file. Now every file shares the same one object.

---

## The rule you asked about — *type* vs *shared object*

- A **blueprint (class)** — `Shelter`, `Dog`, `Cat` — can be pasted into any number of files with `#include`. It **never needs `extern`.** Copying a design around is harmless.
- A **shared object** — `shelter` — must exist exactly once. This is the **only** kind of thing that needs `extern` (the promise, in the header) plus one real definition (in the owner file).

So **Dog and Cat don't need `extern` because they're blueprints**, not because they're "used in few places." `shelter` needs it because it's *one shared object*. If you ever wanted a single shared dog:

```cpp
extern Dog officeDog;   // in a header  — needs extern, because it's now a shared OBJECT
Dog officeDog;          // in one file   — the real one, once
```

...it would need `extern` too, even though it's a Dog. It's **blueprint vs. shared-object** — never "how many files use it."

(And notice `Dog.hpp` already `#include`s `Shelter.hpp`, which pastes in *both* the Shelter blueprint *and* the `extern shelter` promise. That's why `Dog::arrive()` can call `shelter.checkIn()` without writing any `extern` itself.)

---

## Piece 3 — the owner file (this is your `.ino`)

```cpp
// main.cpp
#include "Shelter.hpp"
#include "Dog.hpp"
#include "Cat.hpp"

Shelter shelter;          // THE definition — the one real shelter is born here, once

int main() {
  Dog d; Cat c;
  d.arrive();
  c.arrive();             // shelter.animals == 2
}
```
```csharp
// C# equivalent (Program.cs)
var d = new Dog();
var c = new Cat();
d.Arrive();
c.Arrive();               // Registry.Shelter.Animals == 2
```

`main.cpp` is the **one place** the shared objects are actually created — the same role `Program.cs` plays in C#.

---

## Recap

| Thing | What it is | In C++ | In C# |
|---|---|---|---|
| `Shelter`, `Dog`, `Cat` | blueprints (classes) | define in a header, `#include` anywhere, **no `extern`** | `class Foo { }` |
| `shelter` | one **shared object** | `extern` promise in the header **+** one real line in `main.cpp` | `static Shelter Shelter = new();` |
| `#include "X.hpp"` | paste X's whole text here | — | nearest cousin is `using` (but that's name-lookup, not pasting) |

---

## Your drone, in these exact terms

| Your code | Which kind is it? | C# it's like |
|---|---|---|
| `struct MotorController { ... };` in `MotorController.hpp` | a **blueprint** → no `extern` | `class MotorController { }` |
| `extern MotorController motorController;` in that header | the **promise** of the shared object | *(C# does this for you)* |
| `MotorController motorController;` in the `.ino` | the **one real object**, born once | `static MotorController Motor = new();` |
| a phase calling `motorController.computeMotorMix(...)` | **using** the shared object | `Registry.Motor.ComputeMix(...)` |

The whole reason `extern` exists: **C++ compiles each file blind to the others, so you hand every file a written promise about what's defined elsewhere. C# never shows you this step because it compiles your whole program at once.**

// ============================================================
// CHEAT SHEET -- read this before diving back in.
// ------------------------------------------------------------
// The symbols * and & each do TWO different jobs. Which one depends on whether
// they're in a TYPE (describing a variable) or an EXPRESSION (doing something).
//
//   Cat* p;      * in a TYPE       -> "p IS a pointer" (the noun)
//   p->speak()   * hidden in ->    -> "FOLLOW the pointer" (dereference)
//   Cat& r = c;  & in a TYPE       -> "r IS a reference" (an alias, never null)
//   &cat         & in an EXPRESSION-> "ADDRESS OF cat" (the verb: MAKES a pointer)
//
//   So:  Cat* p = &cat;   means   "p (a pointer)  =  the address of cat"
//        Cat*  is the noun  ("a pointer variable")
//        &cat  is the verb  ("go get the address of cat")
//   And they're opposites in expressions:  &cat makes a pointer, *p follows one,
//   so *(&cat) == cat.
//
// DOT vs ARROW -- how you reach into a thing:
//   have an OBJECT or REFERENCE  -> use  .     (cat.speak())
//   have a POINTER               -> use  ->    (ptr->speak())     // -> is *then .
//
// PASSING into a function:
//   by value      Cat c        -> a COPY; edits don't reach the caller; uses .
//   by reference  Cat& c       -> the ORIGINAL, no copy; edits DO reach caller; uses .
//   const ref     const Cat&   -> the original but READ-ONLY (compiler blocks edits)
//   by pointer    Cat* c       -> the original via an address; can be null; uses ->
//
// ONE INSTANCE:
//   extern  = one GLOBAL shared across files (declare in header, define once)
//   static (local var) = one instance created on first use, hidden in a function
//
// WHAT'S DEMONSTRATED BELOW (in order):
//   Animal/Cat/Dog  = interface + inheritance + virtual / pure-virtual (= 0)
//   Shelter         = storing base pointers, polymorphism, . vs ->
//   greetBy*        = pass by value / pointer / reference (+ const)
//   putCollarOn     = modifying the caller's object via a reference
//   animalFor/lookup= the drone's phaseFor() registry pattern (+ static = one instance)
// ============================================================

#include <iostream>
#include <string>
using namespace std;

// ============================================================
// Animal = the "interface", exactly like IFlightPhase.
// ============================================================
class Animal {
public:
  virtual ~Animal() {}                    // virtual destructor (cleanup habit)

  string collar = "none";                 // a plain, modifiable PROPERTY

  virtual string name()  const = 0;       // "= 0" -> PURE virtual: MUST implement
  virtual string speak() const = 0;       // "= 0" -> MUST implement

  virtual void onArrive() {               // virtual WITH a body -> OPTIONAL to override
    cout << "   (settles quietly into the shelter)\n";
  }
};

// ============================================================
// Cat implements both required methods AND customizes onArrive.
// ============================================================
class Cat : public Animal {               // ": public Animal" = Cat inherits Animal
public:
  string name()  const override { return "Cat"; }
  string speak() const override { return "Meow"; }
  void onArrive() override {               // Cat chooses to override the default
    cout << "   (hides under a blanket)\n";
  }
};

// ============================================================
// Dog implements the required methods but NOT onArrive,
// so it just uses Animal's default one.
// ============================================================
class Dog : public Animal {
public:
  string name()  const override { return "Dog"; }
  string speak() const override { return "Woof"; }
  // no onArrive() here -> inherits Animal's default
};

// ============================================================
// animalFor() -- the SAME pattern as the drone's phaseFor():
// map an enum to one shared object, via a table of pointers.
// ============================================================
enum AnimalKind { CAT, DOG };

Animal* animalFor(AnimalKind kind) {
  // 'static' on a LOCAL variable is the trick that guarantees ONE instance:
  //   - created exactly ONCE, the first time this function runs (lazily)
  //   - lives for the whole program (not destroyed when the function returns)
  //   - every later call reuses that same object
  // This is why phaseFor() does NOT need `extern` -- see the note in main().
  static Cat cat;
  static Dog dog;

  // The table must be POINTERS: you can't make an array of references.
  static Animal* table[] = { &cat, &dog };   // order matches the enum
  return table[kind];                        // hand back a pointer to the one object
}

// ============================================================
// Shelter holds animals as BASE-CLASS POINTERS (Animal*),
// the same way phaseFor() hands back an IFlightPhase*.
// ============================================================
class Shelter {
public:
  void admit(Animal* a) {                 // 'a' is a POINTER to some animal
    animals[count++] = a;
    cout << "Admitting a " << a->name() << ":\n";
    a->onArrive();                        // ARROW: follow the pointer, call onArrive
  }

  void rollCall() {
    for (int i = 0; i < count; i++) {
      Animal* a = animals[i];             // a pointer to one animal
      // SAME line, but each animal answers as its real type = polymorphism
      cout << " - " << a->name() << " says " << a->speak() << "\n";  // ARROW again
    }
  }

  // ==========================================================
  // lookup() -- phaseFor(), but written as a CLASS METHOD.
  // Same idea as the drone's phaseFor(): map an enum to one shared object via a
  // table of pointers. The `static` locals still guarantee exactly ONE of each,
  // created on first use -- living inside a method changes none of that.
  // (In the firmware phaseFor is a free function; a method works identically.)
  // ==========================================================
  Animal* lookup(AnimalKind kind) {
    static Cat cat;                        // one shared Cat, made on first call
    static Dog dog;                        // one shared Dog
    static Animal* table[] = { &cat, &dog };   // table order matches the enum
    return table[kind];                    // return a pointer to the one object
  }

  // ==========================================================
  // THE THREE WAYS TO PASS AN ANIMAL INTO A METHOD
  // ==========================================================

  // 1) BY VALUE: you receive a real Cat OBJECT (a COPY of the original).
  //    You have an object, so you use the DOT. No * and no ->.
  //
  //    WHY/WHEN: simplest and safest. You get your own private copy, so you
  //    can't accidentally change the caller's cat, and nobody can change yours
  //    mid-method. No lifetime worries.
  //    COST: it physically copies the whole object (wasteful for big ones), and
  //    it can't be polymorphic (you can't take an Animal by value -- see #4).
  //    USE FOR: small, cheap-to-copy things, or when you WANT an independent copy.
  void greetByValue(Cat c) {
    c.collar = "RED";   // legal -- but 'c' is a COPY, so this change is LOST
    cout << "by value:     set collar to " << c.collar
         << " (but only on my private copy)\n";        // '.'  (it's an object)
  }

  // 2) BY POINTER: you receive an ADDRESS. You must follow it with the ARROW.
  //
  //    WHY/WHEN -- the reasons you'd reach for a pointer:
  //      * NO COPY: you work on the ORIGINAL, so changes are seen by the caller
  //        and there's no cost even for a huge object.
  //      * POLYMORPHISM: a base pointer (Animal*) can point at a Cat OR a Dog --
  //        this is THE reason the shelter (and the drone) use pointers.
  //      * "MAYBE NOTHING": a pointer can be nullptr to mean "no animal here."
  //        A reference can NEVER be null, so it can't express "optional/absent".
  //      * STORABLE + RE-POINTABLE: you can keep an array of Animal* and change
  //        what each one points at later (that's exactly what `animals[]` and the
  //        drone's phase table do -- swap which object is "current"). You can't
  //        put references in a plain array or re-seat them.
  //      * SHARED / LONG-LIVED: point at one object that outlives this call
  //        (a heap object, or a single shared instance like each phase).
  //    COST: YOU are responsible that the thing still exists (lifetime), and you
  //    should check for null. Slightly noisier syntax (* and ->).
  void greetByPointer(Cat* c) {
    cout << "by pointer:   " << c->speak() << "\n";    // '->' (it's a pointer)
  }

  // 3) BY REFERENCE: you receive an ALIAS to the original (no copy is made).
  //    You use the DOT -- and a BASE reference (Animal&) is still polymorphic,
  //    so this one Animal& parameter works for a Cat OR a Dog.
  //
  //    WHY/WHEN: the "best of both" for passing an EXISTING object -- no copy
  //    (fast) AND polymorphic AND clean dot syntax AND it can't be null, so it's
  //    safer than a pointer WHEN "nothing" isn't a valid answer. This is the
  //    everyday default; use `const Animal&` for read-only, `Animal&` to modify.
  //    Choose a POINTER instead only when you need one of the pointer-only powers
  //    above: null/optional, storing many in a collection, or re-pointing.
  void greetByReference(const Animal& a) {
    // a.collar = "RED";   // <-- WON'T COMPILE: 'const' means read-only. Uncomment
    //                     //     it to see the compiler stop you. This is how you
    //                     //     PROMISE the caller "I will only look, not touch."
    cout << "by reference: " << a.name() << " says " << a.speak() << "\n";  // '.'
  }

  // 3b) BY NON-CONST REFERENCE: an alias you ARE allowed to modify. Change the
  //     collar here and the CALLER's original animal really changes -- no copy,
  //     no pointer syntax, just a dot. (A non-const pointer Animal* would do the
  //     same thing, but you'd write a->collar and could also pass nullptr.)
  void putCollarOn(Animal& a, const string& color) {
    a.collar = color;   // '.'  -- modifies the ORIGINAL the caller passed in
    cout << "putCollarOn: " << a.name() << "'s collar is now " << a.collar << "\n";
  }

  // 4) THE GOTCHA -- why the shelter stores POINTERS, not plain Animals.
  //    You CANNOT write:  void greet(Animal a) { ... }   // by value
  //    Two reasons:
  //      a) Animal has a pure-virtual method (= 0), so it is ABSTRACT --
  //         the compiler refuses to make a plain Animal object at all.
  //      b) Even with a non-abstract base, copying a Cat into an Animal-sized
  //         box would "slice off" the Cat parts and you'd lose polymorphism.
  //    Pointers (Animal*) and references (Animal&) avoid both: they point at
  //    the ORIGINAL Cat, so its real type -- and its Meow -- survive.

private:
  Animal* animals[10];
  int     count = 0;
};

int main() {
  Cat whiskers;                           // a real Cat OBJECT
  Dog rex;                                // a real Dog OBJECT

  Shelter shelter;
  shelter.admit(&whiskers);               // &whiskers = the ADDRESS of whiskers (a pointer)
  shelter.admit(&rex);

  cout << "\n-- roll call --\n";
  shelter.rollCall();

  // dot vs arrow, side by side:
  Cat  onTheCounter;                      // we have the OBJECT
  Cat* ptrToCat = &onTheCounter;          // we have a POINTER to it
  cout << "\ndot  gives: " << onTheCounter.speak()   // '.'  because we have the object
       << "\narrow gives: " << ptrToCat->speak()     // '->' because we have a pointer
       << "\n";

  // the three passing styles:
  cout << "\n-- passing styles --\n";
  shelter.greetByValue(whiskers);         // copies whiskers
  shelter.greetByPointer(&whiskers);      // &whiskers = its address
  shelter.greetByReference(whiskers);     // the original, no copy (Cat)
  shelter.greetByReference(rex);          // SAME method also takes a Dog!

  // does the change STICK? watch whiskers' collar before/after each call:
  cout << "\n-- modifying a property --\n";
  cout << "start:            whiskers.collar = " << whiskers.collar << "\n";

  shelter.greetByValue(whiskers);         // sets collar on a COPY
  cout << "after by-value:   whiskers.collar = " << whiskers.collar
       << "   <- unchanged (we edited a copy)\n";

  shelter.putCollarOn(whiskers, "RED");   // sets collar on the ORIGINAL (reference)
  cout << "after reference:  whiskers.collar = " << whiskers.collar
       << "    <- CHANGED (we edited the original)\n";

  // ----------------------------------------------------------
  // animalFor() -- and proof that 'static' gives ONE instance.
  // ----------------------------------------------------------
  cout << "\n-- animalFor lookup (like phaseFor) --\n";
  Animal* a1 = animalFor(CAT);            // first call: creates the one Cat
  Animal* a2 = animalFor(CAT);            // second call: reuses the SAME Cat
  cout << a1->name() << " says " << a1->speak() << "\n";     // arrow: it's a pointer
  cout << "same object? " << (a1 == a2 ? "YES" : "no")
       << "  (address " << a1 << " == " << a2 << ")\n";

  // the same lookup, but as a METHOD on the shelter object:
  cout << "\n-- shelter.lookup (phaseFor as a class method) --\n";
  Animal* d = shelter.lookup(DOG);        // '.' to call the method, '->' to use the result
  cout << d->name() << " says " << d->speak() << "\n";

  // WHY NO `extern` HERE?
  //   `extern` is for a GLOBAL variable that many .cpp files must share -- it
  //   splits "declare it everywhere" from "define it once" (that's how `shared`
  //   and `motors` work in the firmware).
  //   The phase objects are different: they live as function-LOCAL statics,
  //   hidden inside phaseFor(). No other file can even see them, so there's
  //   nothing to `extern`. `static` already guarantees exactly one -- and even
  //   if phaseFor() is included in many files, C++ guarantees they all share the
  //   SAME single static. Two different mechanisms, same "exactly one" result.
}

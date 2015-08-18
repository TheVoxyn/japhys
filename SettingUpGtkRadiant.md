JAPhys introduces new entities which can be affected by players. By default, GtkRadiant do not support these entities, so new ones must be added to its _entity definition list_.

The new entities JAPhys introduce are:
  * func\_physics
_(This list is subject to change due to early development of JAPhys)_

# Adding the new entities #
  1. Open the _mp\_entities.def_ file found in _GameData/base/scripts_.
  1. Scroll to the bottom of the file, and add:
```
/*QUAKED func_physics (0 .5 .8) ?
===========DESCRIPTION============================
Creates a dynamic physics object.
===========SPAWNFLAGS=============================

===========KEYS&VALUES============================
'model' - .glm or .md3 model to also draw.

'shape' - Physics shape to take up. 0 = box, 1 = sphere, 2 = capsule, 3 = convex shape

'mass' - Mass of the object
*/
```
  1. Save and close _mp\_entities.def_.
  1. Open GtkRadiant, and right-click in the map viewport and check that the new entities are available to use.
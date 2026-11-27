# November 27 upstream synchronization

User policy confirmed on November 27:

- carrot-wip follows upstream/carrot-wip navigation.
- carrot-ssikssik also follows the new upstream navigation, including POSITION
  validity/freshness requirements. Conflicting EV4 0x094/0x380 fallback and
  earlier curve/ramp/junction behavior are replaced, while unrelated local
  features remain independent.
- carrot-dev retains its existing navigation behavior, especially the EV4
  0x094/0x380 section fallback when POSITION is unavailable. The conflicting
  navigation changes from upstream PRs 590 and 591 are excluded on this branch.
  PR 588 build recovery and PR 589 shared CAN-FD signal handling are accepted.

Do not reintroduce excluded navigation behavior in a later merge solely because
both upstream histories are ancestors. Existing branch-specific paddle schema
ordinals must remain stable; do not renumber fields to match stock upstream.
No vehicle installation, settings changes, service restarts or CAN transmission
are part of synchronization. Offline tests do not establish road validation.

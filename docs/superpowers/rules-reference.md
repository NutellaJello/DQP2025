# DECODE 2025-26 Rules Reference
**Sources:** Section 10 Scoring · Section 11 Game Rules V15 · Section 12 Robot Construction Rules V6  
**Format:** Every rule from Sections 10, 11, and 12. Verbatim constraint + violation + design note where applicable.  
**Tags:** `[DESIGN]` = directly shapes robot hardware/software · `[STRATEGY]` = shapes match play · `[ADMIN]` = event/conduct only

---

## 10 SCORING

### 10.1 Match Periods

A MATCH consists of:
- **AUTO:** 30 seconds — robots operate without driver input
- **Transition:** 8 seconds between AUTO and TELEOP (scoring purposes only)
- **TELEOP:** 2 minutes — drivers remotely operate robots

### 10.5 Scoring Overview

Alliances earn MATCH POINTS and RANKING POINTS (RP) for accomplishments during a match: LEAVING the LAUNCH LINE, scoring CLASSIFIED or OVERFLOW ARTIFACTS, scoring ARTIFACTS in the DEPOT, achieving a PATTERN of ARTIFACTS, returning to BASE, and winning or tying matches.

**Assessment timing rules (Section 10.5):**

| ID | When assessed |
|----|---------------|
| A | ARTIFACT scoring (CLASSIFIED/OVERFLOW) — assessed throughout match and until all ARTIFACTS come to rest after match end. ARTIFACTS meeting criteria before TELEOP starts count as AUTO. |
| B | AUTO PATTERN scoring — when all ARTIFACTS come to rest after AUTO ends or TELEOP starts, whichever is first. |
| C | TELEOP PATTERN scoring — when all ROBOTS and ARTIFACTS come to rest after MATCH ends. |
| D | DEPOT scoring — at end of TELEOP when all ROBOTS and ARTIFACTS come to rest. |
| E | LEAVE scoring — at end of AUTO. |
| F | BASE scoring — at end of TELEOP. |

Achievements scored before the match starts, during AUTO-to-TELEOP transition, and after match ends at 0:00 are subject to penalties.

### 10.5.1 ARTIFACT Scoring Criteria

To qualify for CLASSIFIED or OVERFLOW points, an ARTIFACT must enter the GOAL through the open top, exit under the archway, and pass through the diverting SQUARE.
- **CLASSIFIED:** ARTIFACT passes through the SQUARE and transitions directly to the RAMP (does not roll over or bypass ARTIFACTS already on the RAMP).
- **OVERFLOW:** ARTIFACT passes through the SQUARE but does not meet CLASSIFIED criteria (may roll over one or more ARTIFACTS on the RAMP).

ARTIFACTS that do not enter through the open top, do not exit under the archway, or do not pass through the SQUARE do not score as either CLASSIFIED or OVERFLOW.

**DEPOT scoring:** ARTIFACTS must be over the DEPOT (alliance-specific, adjacent to the GOAL). Assessed after match without regard to which alliance placed them. DEPOTs are not protected — either alliance can remove ARTIFACTS from either DEPOT during the match. An ARTIFACT over a DEPOT that is in CONTROL of a ROBOT still qualifies for DEPOT points.

### 10.5.2 PATTERN Scoring Criteria

At the end of AUTO and TELEOP, ARTIFACTS directly on the RAMP score PATTERN points if: (1) the colour of the ARTIFACT at each index matches the MOTIF colour for that index, and (2) the ARTIFACTS are retained by the GATE.

The OBELISK randomisation before the match selects the MOTIF (a series of 2 purple (P) and 1 green (G) in a unique order: GPP, PGP, or PPG), which is repeated 3 times to define the PATTERN colours for each of the 9 indices on the RAMP. Each matching index scores PATTERN points.

### 10.5.3 ROBOT Scoring Criteria

**LEAVE:** A ROBOT must move such that it is no longer over any LAUNCH LINE at the end of AUTO.

**BASE:**
- **Fully returned:** ROBOT must be supported only (directly or transitively) by the TILE in the BASE ZONE.
- **Partially returned:** ROBOT is partially supported by the BASE ZONE TILE and partially by TILES outside the BASE ZONE.
- Support may be transitive through SCORING ELEMENTS or another ROBOT on the BASE ZONE TILE.

### 10.5.4 Point Values

| Scoring Action | AUTO | TELEOP |
|----------------|------|--------|
| LEAVE | 3 | — |
| ARTIFACT — CLASSIFIED | 3 | 3 |
| ARTIFACT — OVERFLOW | 1 | 1 |
| ARTIFACT — DEPOT | — | 1 |
| PATTERN — ARTIFACT matches MOTIF per index | 2 | 2 |
| BASE — Partially returned | — | 5 |
| BASE — Fully returned | — | 10 |
| BASE — Bonus: 2 ROBOTS fully returned | — | 10 |

| RP | Condition | Value |
|----|-----------|-------|
| MOVEMENT RP | Combined LEAVE + BASE points at or above threshold | 1 |
| GOAL RP | Number of ARTIFACTS scored through the SQUARE at or above threshold | 1 |
| PATTERN RP | PATTERN points earned at or above threshold | 1 |
| WIN | Completing match with more MATCH points than opponent | 3 |
| TIE | Completing match with equal MATCH points | 1 |

### 10.5.5 RP Thresholds (Table 10-3)

| RP Type | FIRST Championship | Regional Championships | All Other Events* |
|---------|--------------------|------------------------|-------------------|
| MOVEMENT RP | 21 | 21 | **16** |
| GOAL RP | 67 | 42 | **36** |
| PATTERN RP | 22 | 22 | **18** |

*Premier Events may set their own thresholds. Regional Championship and FIRST Championship thresholds will be announced in Team Updates.

### 10.6 Violations — Penalty Definitions (Table 10-4)

| Penalty | Description |
|---------|-------------|
| **MINOR FOUL** | A credit of **5 points** towards the opponent's MATCH point total |
| **MAJOR FOUL** | A credit of **15 points** towards the opponent's MATCH point total |
| **YELLOW CARD** | A warning issued by the Head REFEREE for egregious ROBOT or team member behaviour. A subsequent YELLOW CARD within the same tournament phase results in a RED CARD. |
| **RED CARD** | A penalty resulting in team being DISQUALIFIED for the MATCH. |
| **DISABLED** | REFEREE instructs team to stop ROBOT, deactivating all outputs for the remainder of the MATCH. |
| **DISQUALIFIED** | Team receives 0 MATCH points and 0 RANKING POINTS in Qualification MATCH; causes ALLIANCE to receive 0 MATCH points in Playoff MATCH. |
| **VERBAL WARNING** | A warning issued by event staff or the Head REFEREE. |
| **ALLIANCE ineligible for RP** | ALLIANCE cannot earn the specified RP for that MATCH, overriding any RP from normal match play or other rule violations. |

### 10.6.1 Duration and Action Terminology

- **MOMENTARY** — durations fewer than approximately **3 seconds**
- **CONTINUOUS** — durations more than approximately **10 seconds**
- **REPEATED** — actions that happen **more than once within a MATCH**

Unless otherwise noted, all penalties are assessed per instance of a rule violation. A single action may violate multiple rules.

### 10.6.1 YELLOW and RED CARD Rules

YELLOW CARDS are additive: a second YELLOW CARD converts to a RED CARD. A team that receives a YELLOW or RED CARD carries a YELLOW CARD into subsequent matches (except as noted below).

**YELLOW CARD clearing:**
- All YELLOW CARDS are cleared at the conclusion of Practice, Qualification, and division Playoff MATCHES.
- VERBAL WARNINGS from the Head REFEREE persist from Qualification through subsequent tournament phases (except when stated otherwise).

**YELLOW/RED CARD application by timing (Table 10-5):**
- Prior to Qualification: Head REFEREE may perpetuate a VERBAL WARNING or YELLOW CARD from pre-qual to first Qualification MATCH for particularly egregious behaviour.
- During Qualification: applied to team's current (or just completed) MATCH, unless SURROGATE (applied to previous Qualification MATCH).
- Between Qualification and Playoff: applied to ALLIANCE's first Playoff MATCH.
- During Playoff: applied to ALLIANCE's current (or just completed) MATCH.

**During Playoff MATCHES:** YELLOW and RED CARDS are assigned to the violating team's entire ALLIANCE. If an ALLIANCE receives 2 YELLOW CARDS, the entire ALLIANCE receives a RED CARD (DISQUALIFICATION for that MATCH).

---

## 11 GAME RULES (G)

### 11.1 Personal Safety

**G101** — Humans stay off field during match `[ADMIN]`  
Humans may only enter the field during pre-match set-up (per G301, G303, G304) and after match is over to retrieve robot.  
*Violation: VERBAL WARNING.*

**G102** — Be careful with arena elements `[ADMIN]`  
Drive team members may not climb on, hang from, manipulate (such that it does not return to its original shape without human intervention), or damage arena elements.  
Drive team members may brace the field perimeter. Moving it out of position is a violation.  
*Violation: VERBAL WARNING. YELLOW CARD if subsequent violations occur during the event.*

---

### 11.2 Conduct

**G201** — Be a good person `[ADMIN]`  
All teams must be civil toward everyone. Egregious examples: assault, threats, harassment, bullying, insulting, swearing at others, yelling in anger.  
*Violation: VERBAL WARNING. YELLOW CARD if subsequent violations occur during the event.*

**G202** — Drive team interactions `[ADMIN]`  
Drive team members may not distract or interfere with the opposing alliance (no taunting or disruptive behaviour).  
*Violation: VERBAL WARNING. YELLOW CARD if subsequent violations occur during the event.*

**G203** — No asking other teams to throw a match `[ADMIN]`  
A team may not encourage an alliance it is not a member of to play beneath its ability.  
*Violation: VERBAL WARNING. RED CARD if subsequent violations occur during the event.*

**G204** — No letting someone coerce you into throwing `[ADMIN]`  
A team, as a result of encouragement from a non-alliance team, may not play beneath its ability.  
*Violation: VERBAL WARNING. RED CARD if subsequent violations occur during the event.*

**G205** — No intentionally losing `[ADMIN]`  
A team may not intentionally lose a match or sacrifice ranking points to lower its own ranking or manipulate other rankings.  
*Violation: VERBAL WARNING. RED CARD if subsequent violations occur during the event.*

**G206** — No colluding to violate rules for ranking points `[ADMIN]`  
A team or alliance may not collude with another team to purposefully violate a rule to influence ranking points.  
*Violation: YELLOW CARD; alliance ineligible for PATTERN and GOAL RPs.*

**G207** — No abusing arena access `[ADMIN]`  
Team members with restricted-area access (event media badges etc.) may not assist, coach, or use signalling devices during a match. Exception: inconsequential infractions and safety.  
*Violation: VERBAL WARNING. YELLOW CARD if subsequent violations occur during the event.*

**G208** — Show up to matches `[ADMIN]`  
If a robot has passed inspection, at least one drive team member must report to the arena and participate in each assigned qualification match.  
*Violation: DISQUALIFIED from current match.*

**G209** — Keep your robot together `[ADMIN]`  
A robot may not intentionally detach or leave a part on the field.  
*Violation: RED CARD.*

**G210** — Don't try to make opponents violate rules `[STRATEGY]`  
Actions clearly aimed at forcing the opponent alliance to violate a rule are not allowed. Rule violations forced in this manner will not result in a penalty to the targeted alliance.  
Example that IS OK: a red robot attempting to access its gate pushes a blue robot into an artifact on the red ramp.  
*Violation: MINOR FOUL. MAJOR FOUL if REPEATED. The ALLIANCE that was forced to break a rule will not be assessed a penalty.*

**G211** — Egregious or exceptional violations `[ADMIN]`  
Head referee may assign a YELLOW or RED CARD for egregious robot actions or team member behaviour at any time, including: inappropriate behaviour per G201, reaching into field grabbing a robot, a single pin >15 seconds, repeatedly descoring scoring elements strategically.  
*Violation: YELLOW or RED CARD.*

**G212** — All teams can play `[ADMIN]`  
A team may not encourage another team to exclude its robot or be disqualified from a qualification match for any reason.  
*Violation: YELLOW CARD. RED CARD if robot does not participate in the match.*

---

### 11.3 Pre-Match

**G301** — Be prompt `[ADMIN]`  
A drive team member may not cause significant delays to match start. Significant = expected start time has passed AND team is not match-ready nor making good-faith effort. Match-ready = robot on field in starting configuration and turned on, drive team in starting positions.  
*Violation (Qual): VERBAL WARNING. MAJOR FOUL for upcoming match if a subsequent violation occurs within the tournament phase. If not match-ready within 2 minutes: DISABLED.*  
*Violation (Playoff): VERBAL WARNING to ALLIANCE. MAJOR FOUL for ALLIANCE's upcoming match if subsequent violation within tournament phase. If not match-ready within 2 minutes: offending team's ROBOT is DISABLED.*

**G302** — Limit what you bring to the field `[ADMIN]`  
Items brought to the field must fit in the alliance area, be worn/held by drive team members, or be an approved accommodation (e.g., step stools, crutches). Items may not: introduce safety hazard, extend >6 ft 6 in above tiles, communicate with anything outside the arena (except medical equipment), block visibility for field staff/audience, or jam/interfere anything in the arena.  
Alignment devices OK for pre-match setup; must not delay match start.  
*Violation: Match will not start. YELLOW CARD if used inappropriately during match.*

**G303** — Robots must come ready to play `[ADMIN]`  
Robot must: not pose hazard, pass inspection, comply with I305 if modified post-inspection, be the only team-provided item left in the field, have correct alliance colour robot signs (R402), be motionless after OpMode initialisation.  
*Violation: Match will not start (quick remedy). DISABLED if not quick remedy. RED CARD if robot not compliant with inspection B or C participates.*

**G304** — Robots must be set up correctly `[DESIGN]`  
Robot must be: over a launch line, touching own alliance's goal or field perimeter, fully contained on own alliance's side (blue = columns A B C; red = columns D E F), not attached/suspended from field elements, in starting configuration, in contact with ≤ allowed pre-loaded artifacts.  
*Violation: Match will not start. DISABLED if not quick remedy.*

**G305** — Teams must select an OpMode `[ADMIN]`  
An OpMode must be selected and initialised on the DRIVER STATION app. If it is an AUTO OpMode, the 30-second AUTO timer must be enabled.  
*Violation: Match will not start. DISABLED if cannot init or situation cannot be remedied quickly.*

---

### 11.4 In-Match

#### 11.4.1 AUTO

**G401** — Let the robot do its thing `[STRATEGY]`  
From randomisation start until end of AUTO, drive team members may not directly or indirectly interact with a robot or operator console. Exceptions: press ▶ at match start, press ■ stop button, personal/console safety.  
Strategic violations (launching multiple scoring elements, operating gate, moving robot a substantial preferred distance) are egregious per G211.  
*Violation: MAJOR FOUL + alliance not eligible for PATTERN points in AUTO if robot LAUNCHES an artifact entering the open top of the GOAL after the interaction and before end of AUTO.*

**G402** — No AUTO opponent interference `[STRATEGY]`  
During AUTO: columns A/B/C = blue side; columns D/E/F = red side. A robot may not contact an opposing alliance robot completely within the opposing side, or disrupt an artifact from its pre-staged location on the opposing side (directly or by launching into it). Launched artifacts that happen to enter opponent side after being deflected by a field element or robot are not penalised.  
*Violation: MAJOR FOUL per robot contact (G402.A). MAJOR FOUL per artifact in G402.B.*

#### 11.4.2 TELEOP

**G403** — Robots motionless between AUTO and TELEOP `[DESIGN]`  
Any powered movement of the robot or mechanisms is not allowed during the AUTO-to-TELEOP transition period. Movement due to inertia, gravity, or de-energising actuators is not a violation. A robot LAUNCHING an artifact during transition is a violation.  
*Violation: MAJOR FOUL.*

**G404** — Robots motionless at end of TELEOP `[STRATEGY]`  
Robots must have no powered movement after the end of TELEOP until head referee signals retrieval.  
*Violation: MINOR FOUL. MAJOR FOUL per artifact if robot LAUNCHES an artifact entering the open top of a GOAL. MAJOR FOUL if robot contacts a GATE.*

#### 11.4.3 Scoring Element

**G405** — Robots use scoring elements as directed `[STRATEGY]`  
A robot may not deliberately use a scoring element to ease or amplify a challenge associated with a field element other than as intended. Examples of violations: intentionally positioning scoring elements to impede opponent access to field elements; intentionally placing scoring elements in inaccessible locations (under ramp or goal); intentionally using a scoring element to hold open the gate.  
*Violation: MAJOR FOUL per scoring element.*

**G406** — Keep scoring elements in bounds `[DESIGN]`  
A robot may not intentionally eject a scoring element from the field (directly or by bouncing off a field element or another robot). Scoring elements that leave the field during scoring attempts are not considered intentional ejections.  
*Violation: MAJOR FOUL per scoring element.*

**G407** — Do not damage scoring elements `[DESIGN]`  
Neither a robot nor a drive team member may damage a scoring element. Reasonable wear (scratching, marking) is expected. Gouging, tearing off pieces, or routinely marking are violations.  
**Design note:** Intake roller, flywheel, belt indexer, and funnel must not have sharp surfaces contacting balls. See also R206. Corrective action (eliminating sharp edges, removing the damaging mechanism, re-inspection) may be required before the robot may compete in subsequent matches.  
*Violation: VERBAL WARNING. MAJOR FOUL if REPEATED. DISABLED if further damage likely.*

**G408** — No more than 3 artifacts controlled simultaneously `[DESIGN]`  
A robot may not simultaneously CONTROL more than 3 ARTIFACTS.  
NOT considered CONTROL: "bulldozing" (inadvertent contact while in robot's path); "deflecting" (ball bouncing into/off robot); inadvertent contact while acquiring from loading zone; artifacts that have been launched and are no longer in contact with robot.  
Excessive violations: simultaneous control of ≥5 artifacts; or ≥3 separate violations in a match with >momentary control of ≥4 artifacts.  
**Design note:** Belt indexer MUST stop accepting balls once 3 are loaded. Use IR sensor or encoder position to enforce. Robot must be designed so that inadvertent control of 4+ is mechanically impossible.  
*Violation: MINOR FOUL per artifact over limit. YELLOW CARD if excessive.*

#### 11.4.4 Robot

**G409** — Robots must be under control `[DESIGN]`  
A robot must not pose an undue hazard to a human or arena element by: (A) the robot or anything it controls disrupting anything outside the field or contacting a human outside the field; (B) dangerous robot operation.  
Robot contact with arena elements outside the field (driver station stand, floor outside field, field wall perimeter outside field) is not a violation. Disrupting the OBELISK is not a violation of this rule.  
*Violation: DISABLED + VERBAL WARNING. YELLOW CARD if REPEATED or if subsequent violations occur during the event.*

**G410** — Robots must stop when instructed `[ADMIN]`  
If instructed to DISABLE by a referee per T202, a drive team member must press the ■ stop button.  
*Violation: MAJOR FOUL if greater-than-MOMENTARY delay. RED CARD if CONTINUOUS.*

**G411** — Robots must be identifiable `[ADMIN]`  
A robot's team number and alliance colour must not become indeterminate.  
*Violation: VERBAL WARNING. MINOR FOUL if subsequent violations occur during the event.*

**G412** — Don't damage the field `[DESIGN]`  
A robot may not damage field elements. Field damage includes: contaminating with liquid or fine solid, damaging tile, causing the gate to bend or break off. Field damage does NOT include: normal gate interaction resulting in gate sticking open; normal interaction with goal that causes it to lift off tiles. (G407 and G412 do not stack.)  
*Violation: VERBAL WARNING. DISABLED if further damage likely. YELLOW CARD for any subsequent damage during the event. Corrective action (eliminating sharp edges, removing the damaging mechanism, re-inspection) may be required before the robot may compete in subsequent matches.*

**G413** — Watch arena interaction `[DESIGN]`  
A robot is prohibited from: grabbing, grasping, attaching to, becoming entangled with, or suspending from arena elements (except scoring elements per G407).  
Gate operation note: robots are expected to push the gate lever DOWN to open. No closing force (e.g., pulling) may be applied to either gate. A robot bumping a stuck-open gate handle to try to close it is NOT a closing force violation.  
**Design note:** Gate push arm must be a simple downward push servo/arm. No hooks, grippers, or mechanisms that could attach to the gate. Corrective action (removing the offending mechanism, re-inspection) may be required before the robot may compete in subsequent matches.  
*Violation: MAJOR FOUL + YELLOW CARD if REPEATED or >momentary. DISABLED if damage likely.*

**G414** — Horizontal expansion limits `[DESIGN]`  
Robots must comply with R105.A (18"×18") during the match. Exception: if over-expansion is due to damage and not used for strategic benefit.  
*Violation: MINOR FOUL. MAJOR FOUL if over-expansion used for strategic benefit including enabling/impeding scoring.*

**G415** — Vertical expansion limits `[DESIGN]`  
Robots must comply with R105 vertical limits. Robots may only expand above 18 in. (45.70 cm) up to 38 in. (96.50 cm) if BOTH conditions are true: (A) during the final 20 seconds of the match, AND (B) when NOT in any launch zone.  
**Design note:** During shooting the robot is in the launch zone — height must stay ≤18 in. at ALL times during normal match play. Option B's ~14 in. build height is compliant.  
*Violation: MINOR FOUL. MAJOR FOUL if over-expansion used for strategic benefit.*

**G416** — Launch only in launch zone `[DESIGN]` `[STRATEGY]`  
Robots may only LAUNCH scoring elements when inside a LAUNCH ZONE or overlapping a LAUNCH LINE.  
LAUNCHED = shot into air, propelled across floor to a desired location or preferred direction, or thrown in a forceful way.  
NOT LAUNCHED (examples): bulldozing; running intake in reverse causing a scoring element to travel a short distance from the robot; robot pushing a scoring element a short distance while herding it across the field.  
**Design note:** Primary throughput bottleneck. Every cycle must return to the launch zone before shooting. Turret flexibility is wasted because robot always shoots from approximately the same field position.  
*Violation: MINOR FOUL per launched scoring element. MAJOR FOUL per launched scoring element if it enters the open top of the GOAL.*

**G417** — Robots only operate own gate `[DESIGN]`  
Robots may not: (A) contact an opposing alliance's gate (directly or transitively through a scoring element); (B) apply any closing force to either gate (directly or transitively through a scoring element). A robot bumping into a stuck-open gate handle is not a closing force violation.  
**Design note:** Gate push arm must only be able to reach own alliance's gate, never opponent's.  
*Violation: MAJOR FOUL + opposing alliance awarded PATTERN RP if G417.A.*

**G418** — Robots may not meddle with artifacts on ramps `[DESIGN]`  
Robots may not contact (directly or transitively through a controlled scoring element) artifacts on a ramp, including their own ramp. Additionally, robots may not: (A) remove an artifact from their own ramp except by operating the gate; (B) remove an artifact from the opponent's ramp by any means.  
Exception: inconsequential and inadvertent contact made by a robot while operating a gate.  
**Design note:** The gate push arm is the only legal method for releasing pre-staged ramp balls. The intake must not be positioned to contact balls still on the ramp.  
*Violation: MAJOR FOUL per artifact. Alliance ineligible for PATTERN RP if G418.A. Opposing alliance awarded PATTERN RP if G418.B.*

**G419** — Launch into own goal only `[STRATEGY]`  
Robots may not: (A) intentionally place or launch artifacts directly onto their own ramp; (B) place or launch artifacts into the opponent's goal or onto the opponent's ramp.  
Intent is for robots to score by launching into the open top of their own goal. No violation for scoring in an opponent's DEPOT.  
*Violation: MAJOR FOUL per artifact + opposing alliance awarded PATTERN RP if G419.B.*

#### 11.4.5 Opponent Interaction

*Note: G420 and G421 are mutually exclusive. If both violated by a single robot-to-robot interaction, only the most punitive penalty is assessed.*

**G420** — Not combat robotics `[DESIGN]`  
A robot may not deliberately functionally impair an opponent robot. Damage or functional impairment due to contact with a tipped-over or disabled opponent robot not perceived as deliberate is not a violation.  
**Design note:** Robot must not have mechanisms intended to damage or impair others. Robust design per R203.K. "Unable to drive" means the driver can no longer drive to a desired location in a reasonable time.  
*Violation: MAJOR FOUL + YELLOW CARD. MAJOR FOUL + RED CARD if opponent robot unable to drive.*

**G421** — No tipping or entangling `[DESIGN]`  
A robot may not deliberately (as perceived by a referee) attach to, tip, or entangle an opponent robot. Tipping as unintended consequence of normal robot-to-robot interaction (single frame hit) is not a violation.  
**Design note:** Avoid wedge-like chassis profiles or any mechanism that could hook under opponent robots. "Unable to drive" means the driver can no longer drive to a desired location in a reasonable time.  
*Violation: MAJOR FOUL + YELLOW CARD. MAJOR FOUL + RED CARD if CONTINUOUS or opponent robot unable to drive.*

**G422** — 3-second pin limit `[STRATEGY]`  
A robot may not PIN an opponent robot for more than 3 seconds. PINNING = preventing movement of opponent by contact (direct or transitive through field element) while opponent is attempting to move.  
Pin count ends when: (A) robots separated ≥2 ft for >3 seconds; (B) either robot moved ≥2 ft from where pin initiated for >3 seconds; (C) pinning robot gets pinned. Pin count pauses when separation/movement criteria are met, and resumes if PINNING ROBOT returns within 2 ft.  
*Violation: MINOR FOUL + additional MINOR FOUL every 3 seconds the situation is not corrected.*

**G423** — No strategies to shut down major gameplay `[STRATEGY]`  
A robot or alliance may not, in the judgement of a referee, isolate or close off any major element of match play for a >momentary duration. Examples: shutting down access to all scoring elements; quarantining an opponent to a small area; quarantining scoring elements out of opponent reach; completely blocking opponent's gate access.  
*Violation: MINOR FOUL + additional MINOR FOUL every 3 seconds.*

**G424** — Gate zone is off limits `[STRATEGY]`  
A robot may not contact (directly or transitively through a scoring element) an opponent robot if either robot is in the opponent's GATE ZONE, regardless of who initiates contact. Exception: a robot in their own alliance's gate zone AND in the opponent's SECRET TUNNEL ZONE is not protected under G424 (G425 applies instead).  
*Violation: MINOR FOUL.*

**G425** — Keep out of opponent's Secret Tunnel Zone `[STRATEGY]`  
A robot in the opponent's SECRET TUNNEL ZONE may not contact (directly or transitively through a scoring element) an opponent robot, regardless of who initiates contact.  
*Violation: MINOR FOUL.*

**G426** — Loading Zone protection `[STRATEGY]`  
A robot may not contact (directly or transitively through a scoring element) an opponent robot while either robot is in the opponent's LOADING ZONE, regardless of who initiates contact.  
*Violation: MINOR FOUL.*

**G427** — Base Zone protection `[STRATEGY]`  
During the last 20 seconds of the match, a robot may not contact (directly or transitively through a scoring element) an opponent robot while either robot is in the opponent's BASE ZONE, regardless of who initiates contact.  
**Strategy note:** Robot should be in own BASE ZONE in last 20 seconds to earn BASE points and be protected.  
*Violation: MAJOR FOUL + opponent robot and any robot fully supported by contacted robot are awarded fully returned-to-BASE points.*

#### 11.4.6 Human

**G428** — No wandering `[ADMIN]`  
Drive team members must remain in their designated ALLIANCE AREA. Drive team members may be anywhere within the alliance area during a match. Drive team members must be staged inside their respective ALLIANCE AREA prior to match start. Drive team members may retrieve scoring elements that have left the field if they can do so without violating G428, G430, and G434. Reintroduction must follow G432.  
*Violation: VERBAL WARNING. MINOR FOUL if subsequent violations occur during the event.*

**G429** — Drive Coaches and other teams: hands off the controls `[ADMIN]`  
A robot shall be operated only by the DRIVERS of that team; DRIVE COACHES may not handle the gamepads. DRIVE COACHES may assist DRIVERS only by: (A) holding the DRIVER STATION device; (B) troubleshooting the DRIVER STATION device; (C) selecting OpModes on the DRIVER STATION app; (D) pressing the INIT button; (E) pressing the ▶ start button; (F) pressing the ■ stop button.  
Exceptions may be made before a match for major conflicts (e.g., religious holidays, major testing, transportation issues).  
*Violation: MAJOR FOUL. YELLOW CARD if greater-than-MOMENTARY.*

**G430** — Drive Coaches, scoring elements are off limits `[ADMIN]`  
DRIVE COACHES may not contact SCORING ELEMENTS, unless for safety purposes.  
*Violation: MINOR FOUL.*

**G431** — Drive teams, watch your reach `[ADMIN]`  
Once a match starts, a DRIVE TEAM member inside the field may not: (A) directly contact a robot; (B) contact a scoring element in contact with a robot; (C) disrupt scoring element scoring; (D) contact a field element.  
Impacting ARTIFACT scoring includes: contacting an ARTIFACT launched by the opponent within the field; contacting an ARTIFACT in the opponent's GOAL; disrupting the scoring of an ARTIFACT on the opponent's RAMP or by operating the opponent's GATE.  
Exceptions are granted for actions that are inadvertent, MOMENTARY, and inconsequential, or concerning safety.  
*Violation: MAJOR FOUL + YELLOW CARD if G431.A. RED CARD and opposing alliance awarded PATTERN RP if G431.C.*

**G432** — Humans may only meddle with artifacts in the Loading Zone `[ADMIN]`  
DRIVE TEAM members may only introduce ARTIFACTS to, remove ARTIFACTS from, or move ARTIFACTS within the LOADING ZONE. Actions must: (A) occur only during TELEOP; (B) not use a tool; (C) not cause an ARTIFACT to enter the LOADING ZONE from elsewhere on the field; (D) not cause an ARTIFACT to leave the LOADING ZONE and enter the rest of the field unless CONTROLLED by a ROBOT (ARTIFACT CONTROL begins when the ROBOT is in the LOADING ZONE and the ARTIFACT is still CONTROLLED when the ROBOT leaves the LOADING ZONE). DRIVE TEAM members may load SCORING ELEMENTS into a ROBOT that is partially or fully in the LOADING ZONE.  
*Violation: MINOR FOUL per ARTIFACT. MAJOR FOUL per ARTIFACT that enters the open top of the GOAL.*

**G433** — Humans may only enter scoring elements `[ADMIN]`  
DRIVE TEAM members may only enter ARTIFACTS (not other items) onto the field.  
*Violation: MINOR FOUL per non-ARTIFACT item entered onto the field.*

**G434** — The Alliance Area has a storage limit `[STRATEGY]`  
During TELEOP, each ALLIANCE may not store more than 6 ARTIFACTS out of play. DRIVE TEAM members making a good-faith effort to immediately enter additional ARTIFACTS back into play is an exception. During AUTO and transition, this rule is not enforced. Upon start of TELEOP, DRIVE TEAM members must make good-faith effort to immediately enter ARTIFACTS until compliant.  
Examples of "out of play": a DRIVE TEAM member holding an ARTIFACT inside or outside the field; DRIVE TEAM member storing an ARTIFACT outside the field.  
*Violation: MINOR FOUL per ARTIFACT over the limit + additional MINOR FOUL per ARTIFACT over the limit every 3 seconds the situation is not corrected.*

---

## 12 ROBOT CONSTRUCTION RULES (R)

*Note: Construction rules apply to the robot as it might be inspected. Match play violations are handled by Section 11.*

**Definitions:**  
- **COMPONENT**: any part in its most basic configuration; cannot be disassembled without damage  
- **MECHANISM**: assembly of components providing specific functionality; can be disassembled into components  
- **COTS item**: standard, unaltered, commercially available part from a vendor  
- **FABRICATED ITEM**: any component/mechanism altered, built, cast, constructed, cut, machined, manufactured, or modified into final form for use on the robot  
- **VENDOR**: a legitimate business source satisfying four criteria (US tax ID or equivalent, not a FIRST team subsidiary, maintains sufficient stock, makes products available to all FTC teams)

---

### 12.1 General Robot Design

**R101** — Starting configuration is 18-inch cube `[DESIGN]`  
In the STARTING CONFIGURATION the robot must be fully self-contained within 18 in. × 18 in. × 18 in. The only exception: pre-loaded scoring elements may extend outside the starting size constraint.  
*Violation: Match will not start (quick remedy). DISABLED if not quick. RED CARD if participates while non-compliant.*

**R102** — Robots may assist in holding starting configuration `[DESIGN]`  
In starting configuration, robots must be fully self-supported (exerts no force on sides/top of a sizing tool). May use: (A) mechanical means while powered-off; (B) OpMode that pre-positions servos/motors to a stationary position. Teams must limit thermal failure risk (e.g., do not stall motors against a hard stop).

**R103** — No robot weight limit  
There is no explicit weight limit. Consider: field tile damage, battery consumption, robot transportation, total robot performance.

**R104** — Keep it together `[DESIGN]`  
Robots may not be designed to intentionally detach components.  
*Violations during match handled by G209.*

**R105** — There are expansion limits `[DESIGN]`  
After match start, robots may expand beyond starting configuration subject to:  
- **R105A (Horizontal):** Must remain within a fixed 18 in. × 18 in. when fully expanded per G414. **Must be physically constrained** — software limits alone are NOT sufficient for inspection. A robot that can mechanically exceed the horizontal limit is in violation even if software prevents it during a match.  
- **R105B (Vertical):** May expand vertically up to 18 in. (45.70 cm). May be physically or software constrained.  
- **R105C (Vertical extended):** Within G415 limits (final 20 seconds AND not in launch zone), may expand vertically up to 38 in. (96.50 cm). May be physically or software constrained.  
Teams must show compliance during inspection (starting config, max horizontal mechanical extension, max vertical extension).

---

### 12.2 Robot Safety & Damage Prevention

**R201** — Do not damage tile floor `[DESIGN]`  
Traction devices must not have surface features known to damage tile floors. High-traction wheels (e.g., AndyMark am-2256, Roughtop am-3309) may not contact tile as part of drive mechanisms. They may be used as part of an intake provided they do not contact the floor.

**R202** — No exposed sharp edges `[DESIGN]`  
Protrusions from the robot and exposed surfaces shall not pose hazards to arena elements (including scoring elements) or people.  
**Design note:** Deburr and round all cut polycarbonate edges, aluminium extrusion edges, and any custom sheet metal.

**R203** — Design robots for safety and fair play `[DESIGN]`  
Robot parts shall not be hazardous materials, unsafe, cause an unsafe condition, or interfere with other robots. Prohibited: shields/curtains obscuring drive team vision or interfering with safe control; speakers/sirens/air horns distracting at match-level volume; devices designed to jam or interfere with remote sensing (vision, acoustic, IR, sonar) of another robot including imagery that mimics 36h11 AprilTags; flammable gases; flames or pyrotechnics; hydraulic fluids or items; mercury switches/contacts; exposed hazardous materials (lead weights must be painted/encapsulated/sealed); high-intensity light sources (must be shrouded; flashing >2Hz invites scrutiny and may be disabled); animal-based materials; devices designed to damage or flip competing robots; devices posing entanglement risk.

**R204** — Scoring elements stay with the field `[DESIGN]`  
Robots must allow removal of scoring elements from the robot and the robot from field elements while powered off. Scoring elements must be quickly, simply, and safely removable.

**R205** — Do not contaminate the field `[DESIGN]`  
Robots may not contain materials that if released would damage the field, other robots, or delay match start due to required clean-up or decontamination. Lubricants may only be used to reduce friction within the robot and must not contaminate the field or other robots. Do not excessively apply grease such that it spins off or drips off during normal operation. Prohibited: loose ballast (sand, kitty litter, ball bearings), liquid/gel materials, tire sealant, graphite powder.

**R206** — Do not damage scoring elements `[DESIGN]`  
Robot elements likely to contact scoring elements shall not pose a significant hazard to the scoring element. Reasonable wear (scratching, marking) is expected; gouging, tearing off pieces, routinely marking are violations.  
**Design note:** All robot surfaces contacting balls (intake roller, belt indexer, flywheel wheels, funnel wings) must be compliant/rounded. No exposed sharp polycarbonate edges or bare metal cuts near ball path.

**R207** — Robots don't use air `[DESIGN]`  
Robots may not use: (A) closed air devices — pneumatic solenoids, cylinders, gas storage vessels, gas springs, compressors, vacuum generators. Air-filled/pneumatic wheels are exempt. (B) Devices that create high-speed airflow — fans designed to move scoring elements. **High-speed flywheels or rollers used to manipulate scoring elements are explicitly NOT high-speed airflow devices and are permitted.**

**R208** — No grabbing the floor `[DESIGN]`  
Robots may not use any mechanism designed to increase downforce by grabbing field surfaces or by generated airflow suction.

---

### 12.3 Fabrication

**R301** — COTS mechanisms are legal but have limits `[DESIGN]`  
COTS MAJOR MECHANISMS purposefully designed to complete a game task are prohibited. Allowed exceptions: (A) COTS drive chassis (individual parts must not violate other rules); (B) COTS MAJOR MECHANISMS created as part of official FTC Starterbots.  
**Design note:** Cannot buy a pre-assembled flywheel launcher or ball intake designed for this game. Must build from individual COTS components (motors, wheels, pulleys, belts, rollers, extrusion) and fabricated items.

**R302** — Legal COTS parts and raw materials can be modified `[DESIGN]`  
Allowed raw materials and legal COTS parts can be modified (drilled, cut, painted, etc.) as long as no other rules are violated. Raw materials include: sheet stock, extruded shapes, metals, plastic, rubber, wood, magnets.

**R303** — COTS must be single degree-of-freedom `[DESIGN]`  
COTS components and mechanisms must not exceed a single DoF. General test: the orientation and position of each component can be predicted from the orientation and position of a single input component.  
Allowed single-DoF COTS: linear slide kit, linear actuator kit, single-speed (non-shifting) gearboxes, pulleys, turntables, lead screws, single-DoF grippers.  
Allowed exceptions (multi-DoF by nature): ratcheting devices (wrenches, bearings), holonomic wheels (omni or mecanum), dead-wheel odometry kits, universal joints/flexible shaft couplers, items that connect structures at variable angles (ball joint linkages, rod ends).  
**Design note:** Mecanum wheels (R303.I) and GoBILDA Pinpoint odometry kit (R303.J) are explicitly legal.

**R304** — Custom parts can be reused year to year  
Fabricated items created before kickoff are permitted.

**R305** — Custom designs and software can be reused year to year  
Robot software and designs created before kickoff are permitted.

**R306** — Current-season scoring elements not allowed for robot construction  
Current season scoring elements (or replicas) are not allowed as part of robot construction or for any other team-supplied scoring elements.

**R307** — During events, work can occur outside pit hours  
A team attending an event may work on or practice with their robot or robot elements outside pit hours. Note: E107 and E108 impose additional restrictions on work done while attending an event.

---

### 12.4 Robot Sign Rules

**R401** — Two robot signs per robot `[ADMIN]`  
Robot signs must be placed in at least 2 separate locations on the robot, on opposite or adjacent surfaces ≥90° apart. All robot surfaces visible to field staff may be used (including top). Requirements: robust material; ≥6.5 in. wide; ≥2.5 in. tall; supported by the robot structure/frame. Must be visible at ≥12 ft.

**R402** — Robot signs indicate alliance `[ADMIN]`  
Each sign must contain a solid red or blue opaque background ≥6.5 in. × 2.5 in. to indicate alliance colour. Cannot be powered to illuminate/reveal alliance colour. Signs must not show opposite alliance colour.

**R403** — Team number on robot signs `[ADMIN]`  
Team numbers: solid opaque white Arabic numerals; 2.25 in. ± 0.5 in. tall; ≥0.25 in. background surrounding numbers; not vertically stacked; robust material; not powered to illuminate/reveal numbers.

---

### 12.5 Motors & Actuators

**R501** — Allowable motors only `[DESIGN]`  
Only these motor actuators are permitted (Table 12-1):

| Motor | Part Number |
|-------|-------------|
| AndyMark NeveRest 12V DC | am-3104, am-3104b |
| AndyMark NeveRest Hex 12V DC | am-3104c |
| goBILDA Yellow Jacket 520x Series 12V DC | 5201, 5202, 5203, 5204 series |
| goBILDA 5000 Series 12V DC | 5000-0002-4008, etc. |
| Modern Robotics/MATRIX 12V DC | 5000-0002-0001 (Discontinued) |
| NFR Products Yuksel 12V DC | NFR-600-100-000 |
| REV Robotics HD Hex 12V DC | REV-41-1291 |
| REV Robotics Core Hex 12V DC | REV-41-1300 |
| Studica Robotics Maverick 12V DC | 75001 |
| SWYFT Robotics SWYFT Spike Motor | SR-MOTOR-DC-01 |
| TETRIX MAX 12V DC | 739530, 39530 (Discontinued) |
| TETRIX MAX TorqueNADO 12V DC | W44260 |

Factory-installed vibration/autofocus motors in COTS computing devices and motors integral to COTS sensors (e.g., LIDAR) do NOT count toward the motor limit in R503.  
Motors sold as gearmotors may be used with or without the provided gearbox or any compatible gearbox.

**R502** — Allowable servos `[DESIGN]`  
Servos must meet: ≤8W mechanical output power @6V; ≤4A stall current @6V. If manufacturer does not provide 6V specs, specs for voltages exceeding 6V may be used.  
Examples of legal servos: AndyMark High-Torque (am-4954), Axon MAX+, DSSERVO 35KG Coreless (DS3235MG), FEETECH FT5335M-FB, goBILDA Dual Mode 2000-0025-0003, REV Smart Servo REV-41-1097, Studica Multi-Mode Smart 75002.  
Linear servos: ≤1A @6V. Examples: Actuonix P8-100-252-12-R, Hitec HLS12-3050-6V, Studica 75014.  
Formula: Mechanical Output Power = 0.25 × (Stall Torque in N·m) × (No Load Speed in rad/s).  
REV Control Hub/Expansion Hub provides 5V to servos; goBILDA Servo Power Injector, REV Servo Power Module, REV Servo Hub, Studica Servo Power Block provide 6V.

**R503** — Maximum 8 motors and 10 servos `[DESIGN]`  
A robot may not have more than 8 motors and 10 servos from the allowable lists per R501 and R502 for ALL mechanisms used in ALL configurations. If a robot has multiple configurations at a single event using different mechanisms, the sum total of all motors and servos must be ≤ the limits.  
**Design note:** Option B uses exactly 8 motors (4 drive + 1 intake + 1 indexer + 2 flywheel) and 3 servos (hood + stopper + gate arm). No margin on motors.

**R504** — Do not modify actuators unless explicitly allowed `[DESIGN]`  
Motors and servos must not be modified except: (A) mounting brackets/output shaft/pinion gears for physical connection; (B) electrical leads trimmed/connectorised; (C) servos modified per manufacturer (reprogramming, continuous rotation); (D) minimal labelling; (E) insulation on electrical terminals; (F) repairs maintaining original performance; (G) manufacturer-recommended maintenance.

**R505** — All actuators must be controlled through approved power regulating devices `[DESIGN]`  
Approved power regulating devices (Table 12-3):

| Device | Part Number | Load Limit |
|--------|-------------|------------|
| goBILDA 6V Servo Power Injector | 3125-0001-0001 | 2 servos per port |
| REV Control Hub / Expansion Hub Motor Ports | REV-31-1153 / REV-31-1595 | 2 motors per port |
| REV Control Hub / Expansion Hub Servo Ports | REV-31-1153 / REV-31-1595 | 2 servos per port |
| REV Servo Power Module | REV-11-1144 | 2 servos per port |
| REV Robotics Servo Hub | REV-11-1855 | 2 servos per port |
| REV SPARKmini | REV-31-1230 | 2 motors per device |
| Studica Servo Power Block | 75005 | 2 servos per port |

**R506** — No relays or alternative electrical actuation `[DESIGN]`  
No application of electromechanical actuation through relays, electromagnets, electrical solenoid actuators, or related systems. Use of relays and electromagnets is also prohibited.

---

### 12.6 Power Distribution

**R601** — Single approved 12V NiMH main battery `[DESIGN]`  
Only one approved 12V NiMH main battery (must have inline 20A ATM mini blade fuse). Legal batteries:

| Battery | Part Number |
|---------|-------------|
| AndyMark Flat Pack 12V | am-5290 |
| goBILDA 12V NiMH Nested | 3100-0012-0020 |
| Matrix 12V 3000mAh NiMH | 14-0014 |
| REV 12V Slim Battery | REV-31-1302 |
| Studica 12V 3000mAh NiMH | 70025 |
| TETRIX MAX 12V 3000mAh NiMH | W39057 |
| WATTOS 12V Battery | WT-NMH1230 |

**R602** — Other batteries only for peripherals and LEDs `[DESIGN]`  
COTS USB battery packs ≤100Wh (27,000mAh @3.7V), with 5V/5A or 12V/5A max output via USB-PD, may be used if: connected only using unmodified COTS cables; charged per recommendations; securely fastened; not supplementing power to robot actuators; electrically isolated from robot power systems. Exceptions for isolation: powered USB hubs and robot controller smartphones.

**R603** — Charge batteries with safe connectors  
Any battery charger must have a corresponding polarised connector. Never charge with alligator clips.

**R604** — Charge batteries at safe rate  
Battery chargers must not exceed 3A average charge current.

**R605** — Batteries are not ballast  
No batteries other than those allowed per R601 and R602 are allowed on the robot, whether or not being used to supply power.

**R606** — Batteries must be securely mounted `[DESIGN]`  
Robot battery must be secured so it will not dislodge during vigorous interaction including tipover. Must be protected from contact with other robots or sharp edges.

**R607** — Electrical connections must be robust and insulated `[DESIGN]`  
All electrical paths may include COTS connectors (Anderson Powerpole, XT30, crimp/quick-connect), splices, COTS flexible/rolling/sliding contacts, COTS slip rings, if: entire electrical pathway uses appropriately gauged/rated elements and all connections are protected from accidental shorts. Teams are strongly encouraged to insulate all exposed electrical terminations.

**R608** — Limit non-battery energy sources `[DESIGN]`  
Non-electrical energy sources stored at match start must come only from: (A) change in altitude of robot centre of gravity; (B) storage achieved by deformation of robot parts including springs, rubber bands, surgical tubing.

**R609** — Main power switch required `[DESIGN]`  
Exactly one main power switch must control all power from battery to all power regulating devices. Must be one of the approved switches and must be accessible. Secondary power switches may be used on the 12V line downstream of the main switch.  
Legal power switches:

| Switch | Part Number |
|--------|-------------|
| AndyMark FTC Power Switch w/ Bracket | am-4969 |
| goBILDA Floodgate Power Switch | 3103-0005-0001 |
| REV Switch Cable and Bracket | REV-31-1387 |
| Studica On/Off Power Switch Kit | 70182 |
| TETRIX R/C Switch Kit | W39129 |
| WATTOS Power Switch Kit | WTS-SW1220 |

Mount switch so it is protected from robot-to-robot contact to avoid inadvertent actuation. Do not mount under access panels, doors, or immediately adjacent to moving components.

**R610** — Fuse ratings must not be altered `[DESIGN]`  
Fuses must not be replaced with higher-rated fuses or per manufacturer specs. Fuses must not exceed rating of those closer to battery. Fuses may be replaced with a smaller rating. Self-resetting fuses (breakers) are NOT allowed. Replaceable fuses must be single-use only.

**R611** — Robot frame is not a wire `[DESIGN]`  
All wiring and electrical devices must be electrically isolated from the robot frame. The frame must not carry electrical current. Grounding control system electronics to frame is permitted ONLY using approved resistive grounding straps (Table 12-6): AndyMark am-4648a, REV REV-31-1269, Swyft SR-Ground-01. Strap must directly connect to a COTS component with XT30 connector AND directly to the robot frame. No robot components or mechanisms may be designed to electrically ground the robot frame to the field.  
Compliance check: unplug battery from main switch; measure >120Ω resistance between (+) or (−) input terminal of main switch assembly and any electrically connected point on the robot.

**R612** — Electrical system must be inspectable `[ADMIN]`  
All power regulating devices, associated wiring, and fuses must be visible for inspection (need not be visible in starting configuration — must be made viewable during inspection process).

**R613** — No high voltage except for LEDs `[DESIGN]`  
Any active electrical item that is not an actuator (R501) or power regulation device (R505) is a CUSTOM CIRCUIT. Custom circuits must not provide regulated output voltages exceeding 5V, except if solely used for powering LEDs (unregulated battery voltage pass-through is OK for LEDs only).

**R614** — Energise power regulating devices as specified `[DESIGN]`  
All power regulating devices must be powered per manufacturer instructions (Table 12-7): REV Control Hub/Expansion Hub — XT30 connectors from robot main battery; REV Servo Power Module — screw terminals from robot main battery; REV Servo Hub — power terminals from robot main battery; REV SPARKmini — Power input from robot main battery; Studica Servo Power Block — JST-VH connector from robot main battery; goBILDA 6V Servo Power Injector — XT30 from robot main battery.

**R615** — Use appropriately sized wire `[DESIGN]`  
Minimum wire sizes (Table 12-8):

| Application | Minimum Wire Size |
|-------------|-------------------|
| 12V main battery power | 18 AWG (19 SWG or 1 mm²) |
| Motor power (unless otherwise listed) | 18 AWG |
| Motor power — TETRIX MAX 12V DC, REV Core Hex (REV-14-1300) | 22 AWG |
| PWM / Servo | 22 AWG (22 SWG or 0.5 mm²) |
| LEDs (5V / 12V) | 22 AWG |
| ≤10A fuse protected circuit | 22 AWG |
| 11–20A fuse protected circuit | 18 AWG |
| Signal-level circuits (≤1A continuous, source incapable of >1A — I2C, DIO, Analog, encoder, RS485) | 28 AWG (29 SWG or 0.08 mm²) |

Integrated wires originally attached to legal COTS devices are exempt. Combining multiple smaller wires in parallel cannot create an equivalent larger wire.

**R616** — Use specified wire colours `[DESIGN]`  
All non-signal-level wiring with constant polarity must be colour-coded: positive (+12V, +5V) = red, yellow, white, brown, or black-with-stripe; negative/common = black or blue. Exception: wires originally attached to legal devices and extensions using the same colour as the manufacturer. Multi-conductor cables not adhering to colour coding may be used if all exposed conductors are re-identified (coloured tape, heat shrink).

**R617** — Powered USB hubs must draw energy from approved sources `[DESIGN]`  
Powered USB hubs may only be powered from: (A) an approved COTS USB battery pack per R602; (B) the 5V auxiliary power port on REV Expansion Hub or REV Control Hub.

**R618** — Do not modify critical power paths `[DESIGN]`  
Custom circuits must not directly alter the power or control pathways between: robot battery and main power switch; main switch and a power regulating device (R609); any two power regulating devices (R613); power regulating devices and actuators. Prohibited: boost converters, buck converters, or any device that alters natural variable DC voltage from battery into a constant DC voltage. Devices that modify actuator control signals or power except those allowed by R505 are prohibited (e.g., goBILDA Servo Travel Tuner is prohibited). Custom high-impedance voltage monitoring or low-impedance current monitoring is acceptable if the effect on power pathways is inconsequential.

**R619** — No mixing power on or between power regulation devices `[DESIGN]`  
Power from ports/connectors on power regulation devices may only be used for devices directly connected to that port/connector. Exception: +5V power from REV Control Hub or Expansion Hub +5V port may be used in conjunction with Analog, Digital, or I2C ports on that device, and to power external devices. 6V power from goBILDA 6V Servo Power Injector, REV Servo Power Module, REV Servo Hub, or Studica Servo Power Block may only power servos. No combining multiple servo port power buses.

---

### 12.7 Control, Command & Signals System

**R701** — Single robot controller `[DESIGN]`  
Robots must be controlled via exactly 1 programmable ROBOT CONTROLLER — the only source of control for robot actuators. Must be: (A) REV Control Hub (REV-31-1595), or (B) approved Android smartphone connected to REV Expansion Hub (REV-31-1153). In addition to A or B, may contain up to 1 additional REV Expansion Hub (REV-31-1153).

**R702** — May not alter coprocessor software `[DESIGN]`  
Modifying software on coprocessors unless explicitly permitted in R703 is not allowed. Firmware updates in binary form provided by the manufacturer may be applied per manufacturer direction. Examples of allowed devices (coprocessor software cannot be changed): Adafruit BNO055 IMU, SparkFun Optical Tracking Odometry Sensor, Digital Chicken Labs OctoQuad.

**R703** — Some vision coprocessors can be programmed `[DESIGN]`  
Programmable vision coprocessors natively supported by the FTC SDK may be programmed. Only Limelight Vision Limelight 3A (LL_3A) is currently supported. Prohibited programmable vision coprocessors: OpenMV Cam, Luxonis OAK-1, LimeLight Vision Limelight 3G.

**R704** — Legal Android smartphones only `[DESIGN]`  
If an Android smartphone is used as robot controller or driver station, it must run Android 7+ (Nougat). Legal devices: Motorola Moto G4 Play, G5, G5 Plus, E4 (USA only — XT1765, XT1765PP, XT1766, XT1767), E5 (XT1920), E5 Play (XT1921).

**R705** — Smartphone robot controller connects via USB `[DESIGN]`  
If used as robot controller, smartphone connects to REV Expansion Hub via integrated micro-USB port using: (A) mini-USB to OTG Micro Cable; or (B) any combination of Mini USB Cable, USB Hub, and OTG Micro adaptor.

**R706** — Wi-Fi bandwidth restricted `[DESIGN]`  
Software with access to the robot network must limit data streamed over Wi-Fi to robot control data, debugging data, and telemetry to/from the robot. No continuous video streams allowed.

**R707** — Configure devices for team number `[ADMIN]`  
Robot controller named `<team number>-RC`; driver station named `<team number>-DS`. Spare devices may add a letter designator (e.g., 12345-A-DS).

**R708** — Do not interfere with robot networks `[ADMIN]`  
During a match, all communications signals must originate only from the robot controller device or driver station device using the robot controller Wi-Fi network. No other devices may attempt to connect, interfere, or alter the robot controller Wi-Fi network during a match (programming laptops and other devices must disconnect before match).

**R709** — No other wireless communication allowed `[DESIGN]`  
No form of wireless communication may be used to communicate to, from, or within the robot except per R706 and R708. Devices that employ visual-spectrum signals (cameras) and non-RF sensors that do not receive human-originated commands (beam-break sensors, IR sensors on robot used to detect field elements) are not wireless communication devices — this rule does not apply to them.

**R710** — Use assigned Wi-Fi bands/channels if requested `[ADMIN]`  
Teams may be asked by the Event Director to use a specific Wi-Fi frequency band or channel on the day of competition. Teams must comply or work with FTA/WTA to find an alternative.

**R711** — Robot controller must be visible for inspection `[ADMIN]`  
The robot controller device must be mounted such that its diagnostic lights (or device screen) can be visible for inspection. Does not need to be visible in starting configuration or during normal match play — must be made viewable during the inspection process.

**R712** — Only specified modifications to core control system devices permitted `[DESIGN]`  
Driver station device, Android-based robot controller device, main and secondary power switches, power regulation devices, fuses, and batteries must not be tampered with, modified, or adjusted in any way. Allowed: (A) wire/cable/signal line connections via standard connection points; (B) fasteners/adhesives to attach devices to console/robot or secure cables; (C) thermal interface material for heat conduction; (D) labelling for identification/purpose/connectivity; (E) jumper location changes; (F) jumper/switch configuration per manufacturer manual; (G) device firmware updates with manufacturer firmware; (H) integral wires on motor controllers and batteries may be cut, stripped, connectorised; (I) devices except batteries may be repaired if performance/specifications remain identical; (J) insulating material on exposed conductors; (K) tape for debris protection; (L) power switch mounting bracket modification or replacement. Repairs that change connector types, device footprint, or provide mechanical enhancements are prohibited.

**R713** — Keep control system software up to date `[DESIGN]`  
Recommended minimum software versions (Table 12-11):

| Device | Software | Recommended Version |
|--------|----------|---------------------|
| REV Control Hub | Control Hub OS | 1.1.2 |
| REV Control Hub | Hub Firmware | 1.8.2 |
| REV Control Hub | Robot Controller App | 11.0 |
| REV Expansion Hub | Hub Firmware | 1.8.2 |
| Android Smartphone (RC) | Robot Controller App | 11.0 |
| Android Smartphone (DS) | Driver Station App | 11.0 |
| REV Driver Hub | Driver Hub OS | 1.2.0 |
| REV Driver Hub | Driver Station App | 11.0 |
| REV Servo Hub | Servo Hub Firmware | 25.0.2 |

Teams may use older versions without affecting inspection status, but field staff may not be able to provide comprehensive support.

**R714** — USB is for vision only `[DESIGN]`  
Only these devices may connect to the robot control system via USB: (A) webcams and optical vision sensors per R715; (B) USB hub or USB switch; (C) a REV Expansion Hub.

**R715** — Use only supported USB vision devices `[DESIGN]`  
Only single image sensor vision devices natively supported by the Robot Controller app are allowed to connect via USB (stereoscopic cameras NOT allowed): (A) all UVC compatible USB webcams (Logitech C270 and related); (B) vision coprocessors allowed per R703. UVC webcams may only use the UVC provided stream/data — no other webcam interfaces or data may be used.

**R716** — Self-contained video recording devices are OK `[DESIGN]`  
GoPro-style cameras are allowed if: used only for non-functional post-match viewing; wireless capability is turned off.

**R717** — Lasers must be safe `[DESIGN]`  
Lasers are not allowed unless: (A) part of a sensor; (B) rated IEC/EN 60825-1 Class I or IEC/EN 62471 Exempt; (C) non-visible spectrum.

**R718** — Configure Android devices appropriately `[DESIGN]`  
REV Control Hub: change Wi-Fi password to non-default. Smartphone users: enable Airplane Mode; Wi-Fi must be enabled and Bluetooth must be disabled. On smartphones and REV Driver Hub: remove all remembered Wi-Fi Direct Groups and Wi-Fi connections, leaving only the robot controller Wi-Fi connection.

---

### 12.8 Pneumatic Systems

**R801** — No pneumatics `[DESIGN]`  
No closed air systems are allowed on FTC robots except for those explicitly listed in R207. (R207 allows: air-filled/pneumatic wheels; flywheels or rollers for ball manipulation.)

---

### 12.9 Operator Console

**R901** — Use only an approved driver station device `[ADMIN]`  
Operator console may have only one approved Android-based driver station device connected and powered on. Must include: (A) REV Driver Hub (REV-31-1596); or (B) approved Android device per R704 with one OTG cable and COTS USB cable for gamepads.

**R902** — Operator console must make touch screen accessible `[ADMIN]`  
The driver station device must be positioned within the operator console so its touch screen is clearly visible during inspection and a match, and functional without additional aides (e.g., mouse).

**R903** — Only limited gamepads supported `[ADMIN]`  
Maximum 2 electrically unmodified gamepads connected at a time from the following list:

| Gamepad | Part Number | Notes |
|---------|-------------|-------|
| Logitech F310 | 940-00010 | |
| Xbox 360 Controller for Windows | 52A-00004 | |
| Sony DualShock 4 Wireless (PS4) | N/A | Wired mode only (USB cable, not Bluetooth) |
| Sony DualSense Wireless (PS5) | N/A | NOT the Edge variant in any configuration |
| Etpark Wired Controller for PS4 | REV-39-1865 | Newer versions may not support all SDK functionality |
| REV Robotics USB PS4 Compatible Gamepad | REV-31-2983 | |
| Quadstick game controller | any model | Xbox 360 Emulation Mode only |

Physical enhancements (back paddles) not modifying electronics are legal. Ferrite cable clips on cables near the USB connector are recommended. Different colour gamepads of the same allowed model are OK.

**R904** — Operator console physical requirements `[ADMIN]`  
Operator console (including all power sources) must not exceed 3 ft wide × 1 ft 2 in deep × 2 ft tall (91.4 cm × 35.5 cm × 61.0 cm), excluding items held/worn by drivers during the match. No hard weight limit, but >20 lbs (~9 kg) invites extra scrutiny.

**R905** — Robot application wireless communication only `[ADMIN]`  
No other form of wireless communications shall be used to communicate to, from, or within the operator console during a match (no Bluetooth, no active wireless network cards). Robot Controller app and Driver Station app connection is the only permitted wireless link.

**R906** — No unsafe or unfair operator consoles `[ADMIN]`  
Operator consoles must not use hazardous materials, be unsafe, cause damage, cause an unsafe condition, distract, or interfere with other drive teams or robots. Distracting sounds (sounds mimicking match sounds, frequent/continuous sounds with no apparent match value) are prohibited.

---

## Quick-Reference: Rules That Directly Constrain Option B Design

| Rule | Hard Constraint | Current Design Status |
|------|----------------|-----------------------|
| G408 | Max 3 balls controlled; robot must make 4+ mechanically impossible | Belt indexer needs hard stop at 3 (IR sensor or encoder) |
| G413 | Gate = push down only; no closing force; no attaching to field elements | Servo push arm — compliant |
| G415 + R105B | Height ≤ 18 in. while in launch zone (i.e., during all normal play) | ~14 in. design — compliant |
| G416 | Launch only from launch zone or launch line | Return-to-zone cycle is core of Option B |
| G417 | No contact with opponent gate directly or through scoring element | Gate arm must only reach own gate |
| G418 | No contact with artifacts on ramp except via own gate | Intake must not reach ramp artifacts |
| G427 | During last 20s, no contact with opponent robot while either is in opponent BASE ZONE | Move to own BASE ZONE before end of match |
| G429 | Drive Coaches cannot touch gamepads — only DRIVERS operate robot | Assign driver roles clearly at events |
| G432 | Artifact loading into robot only from LOADING ZONE during TELEOP only | Human player position and timing discipline |
| G434 | Max 6 artifacts stored out of play at any time during TELEOP | Human player must immediately re-enter balls returned by field staff |
| R101 + R105A | Start in 18 in. cube; ≤ 18 in. × 18 in. horizontal at all times; **mechanical enforcement at inspection** | Funnel wings need physical hinge stops |
| R206 | Robot surfaces contacting balls must not pose hazard to them | Deburr all polycarbonate edges; use compliant rollers |
| R207 | Flywheels for ball manipulation are explicitly permitted | Compliant |
| R301 | Cannot buy a pre-assembled game-specific launcher or intake mechanism | Build from individual COTS components |
| R503 | Max 8 motors, 10 servos | Option B: 8 motors / 3 servos — at motor limit, no spare |
| R709 | IR sensors on robot used to detect field elements are NOT wireless devices | IR ball-count sensor is legal |
| R715 | UVC webcams and Limelight 3A only; stereoscopic cameras not allowed | Single USB webcam — compliant |

---

## Key Scoring Thresholds — Quick Reference (Non-Championship Events)

| RP | Threshold | How to achieve |
|----|-----------|----------------|
| MOVEMENT RP | 16 pts combined LEAVE + BASE | LEAVE (3) + one robot fully BASE (10) = 13 only; need partial BASE on both robots OR LEAVE + full BASE + partial BASE |
| GOAL RP | 36 ARTIFACTS through the SQUARE | ~4 per robot per match at high cycle rates |
| PATTERN RP | 18 PATTERN points | 9 artifacts on ramp matching motif = 9×2=18 AUTO or TELEOP |
| WIN | More match points than opponent | 3 RP |

---

## Key Definitions (Glossary Section 16)

| Term | Definition |
|------|------------|
| ALLIANCE AREA | 96 in. wide × 54 in. deep × infinitely tall volume outside the FIELD |
| ARTIFACT | 5 in. (12.70 cm) nominal Gopher ResisDent polypropylene ball — the only SCORING ELEMENT |
| BASE ZONE | 18 in. × 18 in. ± 0.125 in. infinitely tall volume bounded by alliance-coloured tape |
| CLASSIFIED | ARTIFACT that passes through SQUARE and transitions directly to RAMP |
| CONTINUOUS | Durations more than approximately 10 seconds |
| CONTROL | ROBOT action where SCORING ELEMENT is fully supported by or stuck in/on/under the ROBOT, or ROBOT intentionally pushes it to desired location/direction |
| DEPOT | White tape ~30 in. long spanning the entire length of the GOAL front face at the base |
| GATE | Alliance-specific FIELD element that prevents CLASSIFIED ARTIFACTS from exiting the RAMP |
| GATE ZONE | 2.75 in. wide × 10 in. long infinitely tall volume adjacent to each GATE |
| GOAL | 3-sided structure with horizontal triangular shaped opening at top |
| LAUNCH LINE | White tape bounding 2 triangular LAUNCH ZONES + 2 segments at base of GOAL |
| LAUNCH ZONE | Infinitely tall triangular volumes bounded by LAUNCH LINES and FIELD perimeter |
| LEAVE | ROBOT moves such that it is no longer over any LAUNCH LINE at end of AUTO |
| LOADING ZONE | ~23 in. × 23 in. infinitely tall volume bounded by white tape and FIELD perimeter |
| MAJOR FOUL | 15 points credited to opponent's MATCH point total |
| MINOR FOUL | 5 points credited to opponent's MATCH point total |
| MOMENTARY | Durations fewer than approximately 3 seconds |
| MOTIF | Series of ARTIFACT colours: 2 purple (P) and 1 green (G) in unique order |
| OBELISK | Equilateral triangular prism located just outside FIELD perimeter; randomises MOTIF before match |
| OVERFLOW | ARTIFACT that passes through SQUARE but does not meet CLASSIFIED criteria |
| PATTERN | Scoring achievement — ARTIFACT colour on RAMP matches MOTIF index colour |
| PIN/PINNING | ROBOT preventing movement of opponent ROBOT by contact (direct or transitive) |
| RAMP | Structure that can fit up to 9 CLASSIFIED ARTIFACTS |
| REPEATED | Actions that happen more than once within a MATCH |
| SECRET TUNNEL ZONE | ~46.5 in. long × ~6.125 in. wide infinitely tall volume bounded by alliance tape, GOAL assembly, LOADING ZONE, and FIELD perimeter |
| SQUARE | Location at top of RAMP where ARTIFACT scoring is assessed |

---

*Document updated 2026-05-09 from Section 10, Section 11 V15, and Section 12 V6 of the DECODE Competition Manual (online version). Previous version based on PDF pages of Sections 11 and 12 only.*

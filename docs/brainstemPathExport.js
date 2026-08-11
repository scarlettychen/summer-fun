/**
 * Reference schema for BrainSTEM path export from a canvas/planner UI.
 * Copy into an AutoBuilder-style project (e.g. next to ftcOpmodeGenerator.js).
 *
 * Robot entry points:
 *   PathSpec.parse(jsonString)
 *   PathSpec.fromRaw(resources, rawResId) / PathSpec.fromFile(fileName)
 *   OpmodeCommands.followPath(drive, path)
 *
 * Full contract: docs/PATH_PLANNER_INTEGRATION.md in the SDK repo.
 */

/** @typedef {'inches' | 'meters' | 'cm'} PathUnits */
/** @typedef {'center' | 'corner'} PathOrigin */
/** @typedef {'HOLD_START' | 'HOLD' | 'TANGENT'} HeadingMode */

/**
 * @typedef {Object} Handle
 * @property {number} x
 * @property {number} y
 * @property {boolean} [relative]
 */

/**
 * @typedef {Object} PlannerWaypoint
 * @property {number} x
 * @property {number} y
 * @property {number} [headingDegrees]
 * @property {number} [heading] alias of headingDegrees
 * @property {Handle} [outgoing] cubic out handle (aliases: controlNext, nextHandle)
 * @property {Handle} [incoming] cubic in handle (aliases: controlPrev, prevHandle)
 * @property {number} [maxLinearSpeed] length-unit/s; 0 = dynamic on robot
 * @property {number} [maxVelocity] alias of maxLinearSpeed
 * @property {HeadingMode} [headingMode]
 * @property {boolean} [passPosition] UI-only continuity hint
 * @property {boolean} [tangentHeading]
 */

/**
 * @typedef {Object} BrainstemPathV1
 * @property {'brainstem.path.v1'} [format]
 * @property {string} [name]
 * @property {PathUnits} [units]
 * @property {PathOrigin} [origin]
 * @property {PlannerWaypoint[]} waypoints
 */

/**
 * Build export JSON from AutoBuilder waypoints.
 * Keep generateTrajectory() for simulator preview; call this for robot export.
 *
 * @param {object} opts
 * @param {string} opts.name
 * @param {PlannerWaypoint[]} opts.waypoints  already in the chosen units/origin
 * @param {PathUnits} [opts.units='inches']
 * @param {PathOrigin} [opts.origin='center']
 * @returns {string} pretty JSON for PathSpec.parse / res/raw / RC file drop
 */
export function toBrainstemPathJson({
  name,
  waypoints,
  units = 'inches',
  origin = 'center',
}) {
  const doc = {
    format: 'brainstem.path.v1',
    name: name || 'exported',
    units,
    origin,
    waypoints: waypoints.map((w) => sanitizeWaypoint(w)),
  };
  return JSON.stringify(doc, null, 2);
}

/**
 * Emit TeamCode Java that loads the path at runtime.
 * @param {BrainstemPathV1 | string} docOrJson
 * @param {string} [variableName='path']
 */
export function toBrainstemJava(docOrJson, variableName = 'path') {
  const json = typeof docOrJson === 'string' ? docOrJson : JSON.stringify(docOrJson);
  const escaped = json.replace(/\\/g, '\\\\').replace(/"/g, '\\"').replace(/\n/g, '\\n');
  return (
    `// Generated PathSpec — FieldCoords inches after robot-side unit/origin convert\n` +
    `PathSpec ${variableName} = PathSpec.parse("${escaped}");\n`
  );
}

/** @param {PlannerWaypoint} w */
function sanitizeWaypoint(w) {
  const out = {
    x: w.x,
    y: w.y,
  };
  const heading = w.headingDegrees ?? w.heading;
  if (heading != null) out.headingDegrees = heading;

  const outgoing = w.outgoing || w.controlNext || w.nextHandle;
  if (outgoing) out.outgoing = pickHandle(outgoing);

  const incoming = w.incoming || w.controlPrev || w.prevHandle;
  if (incoming) out.incoming = pickHandle(incoming);

  const speed = w.maxLinearSpeed ?? w.maxVelocity;
  if (speed != null) out.maxLinearSpeed = speed;

  if (w.headingMode) out.headingMode = w.headingMode;
  else if (w.tangentHeading || w.followTangent) out.headingMode = 'TANGENT';

  if (w.passPosition != null) out.passPosition = !!w.passPosition;
  else if (w.passThrough != null) out.passPosition = !!w.passThrough;

  return out;
}

function pickHandle(h) {
  const handle = { x: h.x, y: h.y };
  if (h.relative) handle.relative = true;
  return handle;
}

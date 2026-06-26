/**
 * Parse and validate CoMotion result JSON schema.
 * Provides path clamping for variable-length robot paths.
 */

const SCHEMA_VERSION = "1.0";

/**
 * Get config for robot at timestep t, clamping to last config if path is shorter.
 * @param {Object} robot - Robot object with path and path_length
 * @param {number} t - Timestep index
 * @returns {number[]} Joint configuration
 */
function configAt(robot, t) {
  const path = robot.path || [];
  const pathLength = robot.path_length ?? path.length;
  if (t >= pathLength || t >= path.length) {
    return path.length > 0 ? path[path.length - 1] : [];
  }
  return path[t];
}

/**
 * Parse and validate result JSON. Returns null if invalid.
 * @param {string} text - Raw JSON string
 * @returns {Object|null} Parsed result or null
 */
function parseResult(text) {
  try {
    const data = JSON.parse(text);
    if (!data.robots || !Array.isArray(data.robots)) {
      console.error("Invalid schema: missing or invalid robots array");
      return null;
    }
    if (!data.schema_version) {
      console.warn("No schema_version; assuming v1.0");
      data.schema_version = SCHEMA_VERSION;
    }
    data.timesteps = data.timesteps ?? Math.max(...data.robots.map(r => (r.path || []).length), 0);
    data.obstacles = data.obstacles || [];
    if (
      data.obstacles.length === 0 &&
      Array.isArray(data.benchmark?.context?.obstacles)
    ) {
      data.obstacles = data.benchmark.context.obstacles;
    }
    return data;
  } catch (e) {
    console.error("JSON parse error:", e);
    return null;
  }
}

export { parseResult, configAt, SCHEMA_VERSION };

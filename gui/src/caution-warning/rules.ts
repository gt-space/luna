export const severities = ["warning", "caution", "info"] as const;
export type Severity = typeof severities[number];
type Scalar = string | number | boolean;
type Operator = "eq" | "ne" | "gt" | "gte" | "lt" | "lte" | "exists" | "missing";

export interface Condition {
  path: string;
  operator: Operator;
  value?: Scalar;
}

export interface MessageRule {
  id: string;
  severity: Severity;
  message: string;
  subsystem: string;
  /** Telemetry rules are evaluated only while the stream is fresh. */
  source: "system" | "telemetry";
  all: Condition[];
}

export interface StatusContext {
  connected: boolean;
  hasTelemetry: boolean;
  telemetryFresh: boolean;
  telemetryAgeMs: number | null;
  dataSource: string;
  telemetry: unknown;
}

const comparisons: Record<string, Operator> = { ">": "gt", ">=": "gte", "<": "lt", "<=": "lte", "!=": "ne" };
const forbiddenKeys = ["__proto__", "prototype", "constructor"];
const isObject = (value: unknown): value is Record<string, unknown> =>
  value !== null && typeof value === "object" && !Array.isArray(value);
const isText = (value: unknown): value is string => typeof value === "string" && value.trim().length > 0;
const isScalar = (value: unknown): value is Scalar =>
  typeof value === "string" || typeof value === "boolean" || (typeof value === "number" && Number.isFinite(value));

/** A plain value means equality; an object supplies a comparison. */
function parseCondition(path: string, expected: unknown): Condition {
  const root = path.split(".")[0];
  if (!isText(path) || path.split(".").some(key => !key || forbiddenKeys.includes(key)) ||
      !["connected", "hasTelemetry", "telemetryFresh", "telemetryAgeMs", "dataSource", "telemetry"].includes(root)) {
    throw new Error(`Unknown or invalid field "${path}".`);
  }
  if (isScalar(expected)) return { path, operator: "eq", value: expected };
  if (!isObject(expected) || Object.keys(expected).length !== 1) {
    throw new Error(`"${path}" needs a value or one comparison, such as { "<": 10 }.`);
  }
  const [symbol, value] = Object.entries(expected)[0];
  if (symbol === "exists" && typeof value === "boolean") {
    return { path, operator: value ? "exists" : "missing" };
  }
  if (!Object.prototype.hasOwnProperty.call(comparisons, symbol) || !isScalar(value) ||
      (symbol !== "!=" && typeof value !== "number")) {
    throw new Error(`Invalid comparison for "${path}". Use >, >=, <, <= with a number, != with a value, or exists with true/false.`);
  }
  return { path, operator: comparisons[symbol], value };
}

/** Invalid entries are reported individually; other valid rules remain usable. */
export function loadRules(files: Record<string, string>) {
  const rules: MessageRule[] = [];
  const errors: string[] = [];
  for (const [filename, content] of Object.entries(files).sort()) {
    const name = filename.split("/").pop();
    try {
      const document: unknown = JSON.parse(content);
      if (!Array.isArray(document)) {
        throw new Error('Expected a list of messages: [{ "severity": "caution", "system": "GPS", "description": "No fix", "conditions": { "telemetry.gps.has_fix": false } }].');
      }
      document.forEach((rule: unknown, index: number) => {
        const location = `${name}, message ${index + 1}`;
        try {
          if (!isObject(rule)) throw new Error("Each message must be an object.");
          const extra = Object.keys(rule).filter(key => !["severity", "system", "description", "conditions"].includes(key));
          if (extra.length) throw new Error(`Unrecognized fields: ${extra.join(", ")}. Use severity, system, description, and conditions.`);
          if (!severities.includes(rule.severity as Severity)) throw new Error("severity must be warning, caution, or info.");
          if (!isText(rule.system) || !isText(rule.description)) throw new Error("system and description must contain text.");
          if (!isObject(rule.conditions) || Object.keys(rule.conditions).length === 0) {
            throw new Error("conditions must contain at least one field to check.");
          }
          const all = Object.entries(rule.conditions).map(([path, expected]) => parseCondition(path, expected));
          rules.push({
            id: `${filename}:${index + 1}`,
            severity: rule.severity as Severity,
            subsystem: rule.system,
            message: rule.description,
            source: all.some(condition => condition.path.split(".")[0] === "telemetry") ? "telemetry" : "system",
            all,
          });
        } catch (error) {
          errors.push(`${location}: ${error instanceof Error ? error.message : String(error)}`);
        }
      });
    } catch (error) {
      errors.push(`${name}: ${error instanceof Error ? error.message : String(error)}`);
    }
  }
  if (rules.length === 0) errors.push("No valid message rules are configured.");
  return { rules, errors };
}

function readPath(context: StatusContext, path: string): unknown {
  let value: unknown = context;
  for (const key of path.split(".")) {
    if (forbiddenKeys.includes(key) || value === null || typeof value !== "object" ||
        !Object.prototype.hasOwnProperty.call(value, key)) return undefined;
    value = (value as Record<string, unknown>)[key];
  }
  return value;
}

function matches(context: StatusContext, condition: Condition): boolean {
  const actual = readPath(context, condition.path);
  const present = actual !== undefined && actual !== null &&
    !(typeof actual === "number" && !Number.isFinite(actual));
  if (condition.operator === "exists") return present;
  if (condition.operator === "missing") return !present;
  // Missing data must never accidentally satisfy a comparison (including ne).
  if (!present) return false;
  if (condition.operator === "eq") return actual === condition.value;
  if (condition.operator === "ne") return actual !== condition.value;
  if (typeof actual !== "number" || typeof condition.value !== "number") return false;
  switch (condition.operator) {
    case "gt": return actual > condition.value;
    case "gte": return actual >= condition.value;
    case "lt": return actual < condition.value;
    case "lte": return actual <= condition.value;
    default: return false;
  }
}

export function activeMessages(rules: MessageRule[], context: StatusContext): MessageRule[] {
  return rules.filter(rule => (rule.source === "system" || (context.connected && context.telemetryFresh)) &&
    rule.all.every(condition => matches(context, condition)))
    .sort((a, b) => severities.indexOf(a.severity) - severities.indexOf(b.severity));
}

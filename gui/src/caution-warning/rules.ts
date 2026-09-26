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
  title: string;
  message: string;
  subsystem: string;
  enabled?: boolean;
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

const operators = ["eq", "ne", "gt", "gte", "lt", "lte", "exists", "missing"];
const forbiddenKeys = ["__proto__", "prototype", "constructor"];
const isObject = (value: unknown): value is Record<string, unknown> =>
  value !== null && typeof value === "object" && !Array.isArray(value);
const isText = (value: unknown): value is string => typeof value === "string" && value.trim().length > 0;
const isScalar = (value: unknown): value is Scalar =>
  typeof value === "string" || typeof value === "boolean" || (typeof value === "number" && Number.isFinite(value));

/** Invalid entries are reported individually; other valid rules remain usable. */
export function loadRules(files: Record<string, string>) {
  const rules: MessageRule[] = [];
  const errors: string[] = [];
  const ids = new Set<string>();
  for (const [filename, content] of Object.entries(files).sort()) {
    const name = filename.split("/").pop();
    try {
      const document: unknown = JSON.parse(content);
      if (!isObject(document) || document.version !== 1 || !Array.isArray(document.messages)) {
        throw new Error('Expected version: 1 and a "messages" array.');
      }
      document.messages.forEach((rule: unknown, index: number) => {
        const location = `${name}, message ${index + 1}`;
        if (!isObject(rule) || ![rule.id, rule.title, rule.message, rule.subsystem].every(isText) ||
            !severities.includes(rule.severity as Severity) ||
            !["system", "telemetry"].includes(rule.source as string) ||
            (rule.enabled !== undefined && typeof rule.enabled !== "boolean") ||
            !Array.isArray(rule.all) || rule.all.length === 0) {
          errors.push(`${location}: invalid message fields or empty conditions.`);
          return;
        }
        const invalidCondition = rule.all.some((condition: unknown) => {
          if (!isObject(condition) || !isText(condition.path) ||
              condition.path.split(".").some(key => !key || forbiddenKeys.includes(key)) ||
              !operators.includes(condition.operator as string)) return true;
          const root = condition.path.split(".")[0];
          if (!["connected", "hasTelemetry", "telemetryFresh", "telemetryAgeMs", "dataSource", "telemetry"].includes(root)) return true;
          if (root === "telemetry" && rule.source !== "telemetry") return true;
          if (["exists", "missing"].includes(condition.operator as string)) return false;
          if (["gt", "gte", "lt", "lte"].includes(condition.operator as string)) {
            return typeof condition.value !== "number" || !Number.isFinite(condition.value);
          }
          return !isScalar(condition.value);
        });
        if (invalidCondition) {
          errors.push(`${location}: invalid condition path, operator, or value.`);
        } else if (ids.has(rule.id as string)) {
          errors.push(`${location}: duplicate id "${rule.id}".`);
        } else {
          ids.add(rule.id as string);
          rules.push(rule as unknown as MessageRule);
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
  return rules.filter(rule => rule.enabled !== false &&
    (rule.source === "system" || (context.connected && context.telemetryFresh)) &&
    rule.all.every(condition => matches(context, condition)))
    .sort((a, b) => severities.indexOf(a.severity) - severities.indexOf(b.severity) || a.id.localeCompare(b.id));
}

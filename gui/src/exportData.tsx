import { SERVER_PORT } from "./appdata";
import { serverIp, sessionId } from "./comm";

// sends a generic command to the servers
export async function sendExportLogCommand(body: object): Promise<string> {
  const response = await fetch(`http://${serverIp()}:${SERVER_PORT}/operator/export-log`, {
    headers: new Headers({
      'Authorization': sessionId() as string,
      'Content-Type': 'application/json;charset=utf-8'
    }),
    method: 'POST',
    body: JSON.stringify(body),
  });
  return response.text();
}

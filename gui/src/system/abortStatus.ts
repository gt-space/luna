import type { AbortTransitionStatus } from '../comm';

export interface AbortRequest { id: string; stage: string; time: number; error?: string }

export function abortStatus(transition: AbortTransitionStatus | null, request: AbortRequest | null, now: number, live: boolean, connected: boolean) {
  const confirmed = live && connected && transition?.phase === 'confirmed' && !transition.unsynchronized && !request;
  const label = request?.error ? `FAILED: ${request.error}`
    : request && now - request.time > 7000 ? 'FAILED / TIMEOUT: no matching Flight confirmation'
    : request ? 'PREPARING (request pending)'
    : (transition?.phase || 'unsent').replace('_', ' ').toUpperCase();
  return { confirmed, label };
}

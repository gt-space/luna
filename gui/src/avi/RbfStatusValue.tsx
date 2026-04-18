import { rbfStatusTextColor } from "./rbfDisplay";

export function RbfStatusValue(props: { text: string }) {
  const col = rbfStatusTextColor(props.text);
  return (
    <div class="adc-data-value" style={col ? { color: col } : undefined}>
      {props.text}
    </div>
  );
}

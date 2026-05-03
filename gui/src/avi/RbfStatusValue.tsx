import { rbfStatusTextColor } from "./rbfDisplay";

export function RbfStatusValue(props: { text: string }) {
  return (
    <div
      class="adc-data-value"
      style={{
        color: rbfStatusTextColor(props.text),
      }}
    >
      {props.text}
    </div>
  );
}

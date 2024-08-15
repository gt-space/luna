import { Component } from "solid-js";
import { createSignal, For } from "solid-js";
import { dndzone } from "solid-dnd-directive";
import { Device } from "../devices";

declare module "solid-js" {
  namespace JSX {
    interface Directives {
      dndzone: { items: Accessor<{ id: number; }[]>; };
    }
    interface CustomEvents {
      consider: Event;
      finalize: Event;

    }
  }
}

const DragAndDrop: Component<{sensors: Device[], row: Function}> = (props) => {
  var fake = dndzone
  let sensorDisplays: {id: number, name: string, value: number, unit: string, offset: number}[] = [];
  for (let i = 0; i < props.sensors.length; ++i) {
    let sensor = props.sensors[i];
    sensorDisplays.push({'id': i, 'name': sensor.name, "value": sensor.value, "unit": sensor.unit, "offset": sensor.offset});
  }
  const [items, setItems] = createSignal(
    sensorDisplays
  );
  function handleDndEvent(e: any) {
    const { items: newItems } = e.detail;
    setItems(newItems);
  }
  return (
    <div style="flex: 1; padding: 10px"
      use:dndzone={{ items }}
      on:consider={handleDndEvent}
      on:finalize={handleDndEvent}
    >
      <For each={items()}>
        {(item) => props.row(item.name, item.value, item.unit, item.offset)}
      </For>
    </div>
  );
}
export default DragAndDrop
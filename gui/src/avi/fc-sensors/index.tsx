/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import FcSensors from "./fc-sensors";

render(() => <FcSensors/>, document.getElementById("root") as HTMLElement);
/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import Sensors from "./Sensors";

render(() => <Sensors/>, document.getElementById("root") as HTMLElement);
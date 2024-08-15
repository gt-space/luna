/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import Valves from "./Valves";

render(() => <Valves/>, document.getElementById("root") as HTMLElement);
/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import Taskbar from "./TaskBar";

render(() => <Taskbar />, document.getElementById("root") as HTMLElement);

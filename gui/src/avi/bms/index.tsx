/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import BMS from "./bms";

render(() => <BMS/>, document.getElementById("root") as HTMLElement);
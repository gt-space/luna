/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import IGNITER from "./igniter";

render(() => <IGNITER/>, document.getElementById("root") as HTMLElement);
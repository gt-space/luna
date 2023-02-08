/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import System from "./System";

render(() => <System/>, document.getElementById("root") as HTMLElement);

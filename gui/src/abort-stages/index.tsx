/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import AbortStages from "./AbortStages";

render(() => <AbortStages/>, document.getElementById("root") as HTMLElement);
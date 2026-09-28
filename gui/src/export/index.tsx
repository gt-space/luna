/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import ExportDisplay from "./ExportDisplay";

render(() => <ExportDisplay/>, document.getElementById("root") as HTMLElement);

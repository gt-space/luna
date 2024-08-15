/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import Plotter from "./Plotter";

render(() => <Plotter/>, document.getElementById("root") as HTMLElement);
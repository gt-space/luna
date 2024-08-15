/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import Sequences from "./Sequences";

render(() => <Sequences/>, document.getElementById("root") as HTMLElement);
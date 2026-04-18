/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import RBF from "./rbf";

render(() => <RBF />, document.getElementById("root") as HTMLElement);
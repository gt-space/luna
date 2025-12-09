/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import SAM from "./sam";

render(() => <SAM/>, document.getElementById("root") as HTMLElement);
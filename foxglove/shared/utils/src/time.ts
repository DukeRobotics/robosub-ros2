import { Time } from "@foxglove/extension";

export const secToNsec = (sec: number): number => sec * 1e9;
export const nsecToSec = (nsec: number): number => nsec / 1e9;
export const timeToNsec = (time: Time): number => secToNsec(time.sec) + time.nsec;

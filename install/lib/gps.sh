#!/usr/bin/env bash

pick_serial_by_id() {
  local default_device="${1:-}"
  local by_id_dir="${SERIAL_BY_ID_DIR:-/dev/serial/by-id}"
  local candidates=()
  local choice
  local i=1

  if [ -d "$by_id_dir" ]; then
    while IFS= read -r path; do
      candidates+=("$path")
    done < <(find "$by_id_dir" -maxdepth 1 -type l | sort)
  fi

  if [ "${#candidates[@]}" -eq 0 ]; then
    error "No USB serial device found in $by_id_dir"
    return 1
  fi

  echo ""
  info "Detected USB serial device(s):"
  for path in "${candidates[@]}"; do
    if [ -L "$path" ]; then
      echo "  ${i}) $path -> $(readlink -f "$path")"
    else
      echo "  ${i}) $path"
    fi
    i=$((i + 1))
  done

  local default_idx=""
  if [ -n "$default_device" ]; then
    for i in "${!candidates[@]}"; do
      if [ "${candidates[$i]}" = "$default_device" ]; then
        default_idx="$((i + 1))"
        break
      fi
    done
  fi

  prompt "$MSG_CHOICE" "${default_idx:-1}"
  choice="$REPLY"

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#candidates[@]}" ]; then
    error "Invalid USB serial device selection"
    return 1
  fi

  REPLY="${candidates[$((choice - 1))]}"
}

preset_key_loaded() {
  local wanted="${1:?preset_key_loaded: missing key}"
  local key

  [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ] || return 1

  for key in "${STATE_PARSED_KEYS[@]}"; do
    [ "$key" = "$wanted" ] && return 0
  done

  return 1
}

configure_gps() {
  step "Universal GNSS configuration"

  : "${GNSS_BACKEND:=universal}"
  : "${GNSS_STATUS_SOURCE:=universal}"
  : "${GNSS_STACK:=universal}"
  : "${GNSS_TRANSPORT:=serial}"
  : "${GNSS_RECEIVER_FAMILY:=auto}"
  : "${GNSS_SERIAL_DEVICE:=}"
  : "${GNSS_SERIAL_BAUD:=}"

  local serial_preconfigured=false
  local baud_preconfigured=false
  local probe_mode="ask"
  local receiver_family
  local probe_backend
  local probe_protocol
  local connection
  local probe_port=""
  local default_baud="921600"

  case "${GNSS_BACKEND:-}" in
    ublox|unicore|nmea)
      if [[ -z "${GNSS_RECEIVER_FAMILY:-}" || "${GNSS_RECEIVER_FAMILY:-}" == "auto" ]]; then
        GNSS_RECEIVER_FAMILY="${GNSS_BACKEND}"
      fi
      ;;
  esac

  if [[ "$(effective_gnss_backend "${GNSS_BACKEND:-universal}")" == "disabled" ]]; then
    info "Direct GNSS configuration disabled for HARDWARE_BACKEND=${HARDWARE_BACKEND:-mowgli}"
    return 0
  fi

  if [[ "${PRESET_LOADED:-false}" == "true" ]]; then
    if [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ]; then
      if preset_key_loaded GNSS_SERIAL_DEVICE; then
        serial_preconfigured=true
      fi
      if preset_key_loaded GNSS_SERIAL_BAUD; then
        baud_preconfigured=true
      fi
    else
      if [[ -n "${GNSS_SERIAL_DEVICE:-}" ]]; then
        serial_preconfigured=true
      fi
      if [[ -n "${GNSS_SERIAL_BAUD:-}" ]]; then
        baud_preconfigured=true
      fi
    fi
  fi

  GNSS_BACKEND="universal"
  GNSS_STACK="universal"
  GNSS_STATUS_SOURCE="universal"
  GNSS_TRANSPORT="serial"
  receiver_family="$(normalize_gnss_receiver_family "${GNSS_RECEIVER_FAMILY:-auto}")"
  GNSS_RECEIVER_FAMILY="$receiver_family"

  connection="$(gnss_connection_from_serial_device "${GNSS_SERIAL_DEVICE:-}")"
  if [[ "$serial_preconfigured" != "true" || -z "${GNSS_SERIAL_DEVICE:-}" ]]; then
    local connection_default="2"
    [[ "$connection" == "usb" ]] && connection_default="1"
    echo ""
    echo "$MSG_GPS_CONNECTION"
    echo "  1) USB"
    echo "  2) UART"
    prompt "$MSG_CHOICE" "$connection_default"

    case "$REPLY" in
      1)
        connection="usb"
        pick_serial_by_id "${GNSS_SERIAL_DEVICE:-}" || return 1
        GNSS_SERIAL_DEVICE="$REPLY"
        ;;
      2)
        connection="uart"
        pick_uart_port "${GNSS_SERIAL_DEVICE:-/dev/ttyAMA4}"
        GNSS_SERIAL_DEVICE="$REPLY"
        ;;
      *)
        error "$MSG_GPS_INVALID_CONNECTION"
        return 1
        ;;
    esac
  else
    info "Universal GNSS device pre-configured: ${GNSS_SERIAL_DEVICE}"
    probe_mode="auto"
  fi

  default_baud="${GNSS_SERIAL_BAUD:-921600}"
  probe_port="${GNSS_SERIAL_DEVICE:-}"
  case "$receiver_family" in
    ublox)   probe_backend="ublox" ;;
    unicore) probe_backend="unicore" ;;
    *)       probe_backend="gps" ;;
  esac
  probe_protocol="$(gnss_receiver_family_to_gps_protocol "$receiver_family")"

  if [[ "$baud_preconfigured" != "true" && -n "$probe_port" ]]; then
    GPS_PROTOCOL="$probe_protocol"
    prompt_or_probe_baud "$probe_port" "$probe_backend" "$probe_protocol" "$default_baud" "$probe_mode"
    GNSS_SERIAL_BAUD="$REPLY"
    GPS_BAUD="$GNSS_SERIAL_BAUD"
    maybe_upgrade_unicore_baud "$probe_port" "$GNSS_SERIAL_BAUD" "$probe_mode"
    maybe_upgrade_ublox_baud "$probe_port" "$GNSS_SERIAL_BAUD" "$probe_mode"
    GNSS_SERIAL_BAUD="${GPS_BAUD:-$GNSS_SERIAL_BAUD}"
  elif [[ -z "${GNSS_SERIAL_BAUD:-}" ]]; then
    GNSS_SERIAL_BAUD="$default_baud"
  fi

  echo ""
  info "Universal GNSS : receiver_family=$GNSS_RECEIVER_FAMILY transport=$GNSS_TRANSPORT device=$GNSS_SERIAL_DEVICE baud=$GNSS_SERIAL_BAUD"
  return 0
}

run_gps_configuration_step() {
  configure_gps
}

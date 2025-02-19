/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * schema-defined QAPI visitor functions
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "qemu-common.h"
#include "qga-qapi-visit.h"

static void visit_type_GuestAgentCommandInfo_fields(Visitor *m, GuestAgentCommandInfo **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_str(m, &(*obj)->name, "name", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->enabled, "enabled", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->success_response, "success-response", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestAgentCommandInfo(Visitor *m, GuestAgentCommandInfo **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestAgentCommandInfo", name, sizeof(GuestAgentCommandInfo), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestAgentCommandInfo_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestAgentCommandInfoList(Visitor *m, GuestAgentCommandInfoList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestAgentCommandInfoList *native_i = (GuestAgentCommandInfoList *)i;
        visit_type_GuestAgentCommandInfo(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestAgentInfo_fields(Visitor *m, GuestAgentInfo **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_str(m, &(*obj)->version, "version", &err);
    if (err) {
        goto out;
    }
    visit_type_GuestAgentCommandInfoList(m, &(*obj)->supported_commands, "supported_commands", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestAgentInfo(Visitor *m, GuestAgentInfo **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestAgentInfo", name, sizeof(GuestAgentInfo), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestAgentInfo_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestAgentInfoList(Visitor *m, GuestAgentInfoList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestAgentInfoList *native_i = (GuestAgentInfoList *)i;
        visit_type_GuestAgentInfo(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestFileRead_fields(Visitor *m, GuestFileRead **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_int(m, &(*obj)->count, "count", &err);
    if (err) {
        goto out;
    }
    visit_type_str(m, &(*obj)->buf_b64, "buf-b64", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->eof, "eof", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestFileRead(Visitor *m, GuestFileRead **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestFileRead", name, sizeof(GuestFileRead), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestFileRead_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestFileReadList(Visitor *m, GuestFileReadList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestFileReadList *native_i = (GuestFileReadList *)i;
        visit_type_GuestFileRead(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestFileWrite_fields(Visitor *m, GuestFileWrite **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_int(m, &(*obj)->count, "count", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->eof, "eof", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestFileWrite(Visitor *m, GuestFileWrite **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestFileWrite", name, sizeof(GuestFileWrite), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestFileWrite_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestFileWriteList(Visitor *m, GuestFileWriteList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestFileWriteList *native_i = (GuestFileWriteList *)i;
        visit_type_GuestFileWrite(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestFileSeek_fields(Visitor *m, GuestFileSeek **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_int(m, &(*obj)->position, "position", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->eof, "eof", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestFileSeek(Visitor *m, GuestFileSeek **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestFileSeek", name, sizeof(GuestFileSeek), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestFileSeek_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestFileSeekList(Visitor *m, GuestFileSeekList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestFileSeekList *native_i = (GuestFileSeekList *)i;
        visit_type_GuestFileSeek(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestFsfreezeStatusList(Visitor *m, GuestFsfreezeStatusList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestFsfreezeStatusList *native_i = (GuestFsfreezeStatusList *)i;
        visit_type_GuestFsfreezeStatus(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestFsfreezeStatus(Visitor *m, GuestFsfreezeStatus *obj, const char *name, Error **errp)
{
    visit_type_enum(m, (int *)obj, GuestFsfreezeStatus_lookup, "GuestFsfreezeStatus", name, errp);
}

void visit_type_GuestIpAddressTypeList(Visitor *m, GuestIpAddressTypeList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestIpAddressTypeList *native_i = (GuestIpAddressTypeList *)i;
        visit_type_GuestIpAddressType(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestIpAddressType(Visitor *m, GuestIpAddressType *obj, const char *name, Error **errp)
{
    visit_type_enum(m, (int *)obj, GuestIpAddressType_lookup, "GuestIpAddressType", name, errp);
}

static void visit_type_GuestIpAddress_fields(Visitor *m, GuestIpAddress **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_str(m, &(*obj)->ip_address, "ip-address", &err);
    if (err) {
        goto out;
    }
    visit_type_GuestIpAddressType(m, &(*obj)->ip_address_type, "ip-address-type", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->prefix, "prefix", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestIpAddress(Visitor *m, GuestIpAddress **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestIpAddress", name, sizeof(GuestIpAddress), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestIpAddress_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestIpAddressList(Visitor *m, GuestIpAddressList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestIpAddressList *native_i = (GuestIpAddressList *)i;
        visit_type_GuestIpAddress(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestNetworkInterface_fields(Visitor *m, GuestNetworkInterface **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_str(m, &(*obj)->name, "name", &err);
    if (err) {
        goto out;
    }
    visit_optional(m, &(*obj)->has_hardware_address, "hardware-address", &err);
    if (!err && (*obj)->has_hardware_address) {
        visit_type_str(m, &(*obj)->hardware_address, "hardware-address", &err);
    }
    if (err) {
        goto out;
    }
    visit_optional(m, &(*obj)->has_ip_addresses, "ip-addresses", &err);
    if (!err && (*obj)->has_ip_addresses) {
        visit_type_GuestIpAddressList(m, &(*obj)->ip_addresses, "ip-addresses", &err);
    }
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestNetworkInterface(Visitor *m, GuestNetworkInterface **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestNetworkInterface", name, sizeof(GuestNetworkInterface), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestNetworkInterface_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestNetworkInterfaceList(Visitor *m, GuestNetworkInterfaceList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestNetworkInterfaceList *native_i = (GuestNetworkInterfaceList *)i;
        visit_type_GuestNetworkInterface(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestLogicalProcessor_fields(Visitor *m, GuestLogicalProcessor **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_int(m, &(*obj)->logical_id, "logical-id", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->online, "online", &err);
    if (err) {
        goto out;
    }
    visit_optional(m, &(*obj)->has_can_offline, "can-offline", &err);
    if (!err && (*obj)->has_can_offline) {
        visit_type_bool(m, &(*obj)->can_offline, "can-offline", &err);
    }
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestLogicalProcessor(Visitor *m, GuestLogicalProcessor **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestLogicalProcessor", name, sizeof(GuestLogicalProcessor), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestLogicalProcessor_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestLogicalProcessorList(Visitor *m, GuestLogicalProcessorList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestLogicalProcessorList *native_i = (GuestLogicalProcessorList *)i;
        visit_type_GuestLogicalProcessor(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestDiskBusTypeList(Visitor *m, GuestDiskBusTypeList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestDiskBusTypeList *native_i = (GuestDiskBusTypeList *)i;
        visit_type_GuestDiskBusType(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestDiskBusType(Visitor *m, GuestDiskBusType *obj, const char *name, Error **errp)
{
    visit_type_enum(m, (int *)obj, GuestDiskBusType_lookup, "GuestDiskBusType", name, errp);
}

static void visit_type_GuestPCIAddress_fields(Visitor *m, GuestPCIAddress **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_int(m, &(*obj)->domain, "domain", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->bus, "bus", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->slot, "slot", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->function, "function", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestPCIAddress(Visitor *m, GuestPCIAddress **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestPCIAddress", name, sizeof(GuestPCIAddress), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestPCIAddress_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestPCIAddressList(Visitor *m, GuestPCIAddressList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestPCIAddressList *native_i = (GuestPCIAddressList *)i;
        visit_type_GuestPCIAddress(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestDiskAddress_fields(Visitor *m, GuestDiskAddress **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_GuestPCIAddress(m, &(*obj)->pci_controller, "pci-controller", &err);
    if (err) {
        goto out;
    }
    visit_type_GuestDiskBusType(m, &(*obj)->bus_type, "bus-type", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->bus, "bus", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->target, "target", &err);
    if (err) {
        goto out;
    }
    visit_type_int(m, &(*obj)->unit, "unit", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestDiskAddress(Visitor *m, GuestDiskAddress **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestDiskAddress", name, sizeof(GuestDiskAddress), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestDiskAddress_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestDiskAddressList(Visitor *m, GuestDiskAddressList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestDiskAddressList *native_i = (GuestDiskAddressList *)i;
        visit_type_GuestDiskAddress(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestFilesystemInfo_fields(Visitor *m, GuestFilesystemInfo **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_str(m, &(*obj)->name, "name", &err);
    if (err) {
        goto out;
    }
    visit_type_str(m, &(*obj)->mountpoint, "mountpoint", &err);
    if (err) {
        goto out;
    }
    visit_type_str(m, &(*obj)->type, "type", &err);
    if (err) {
        goto out;
    }
    visit_type_GuestDiskAddressList(m, &(*obj)->disk, "disk", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestFilesystemInfo(Visitor *m, GuestFilesystemInfo **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestFilesystemInfo", name, sizeof(GuestFilesystemInfo), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestFilesystemInfo_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestFilesystemInfoList(Visitor *m, GuestFilesystemInfoList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestFilesystemInfoList *native_i = (GuestFilesystemInfoList *)i;
        visit_type_GuestFilesystemInfo(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestMemoryBlock_fields(Visitor *m, GuestMemoryBlock **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_uint64(m, &(*obj)->phys_index, "phys-index", &err);
    if (err) {
        goto out;
    }
    visit_type_bool(m, &(*obj)->online, "online", &err);
    if (err) {
        goto out;
    }
    visit_optional(m, &(*obj)->has_can_offline, "can-offline", &err);
    if (!err && (*obj)->has_can_offline) {
        visit_type_bool(m, &(*obj)->can_offline, "can-offline", &err);
    }
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlock(Visitor *m, GuestMemoryBlock **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestMemoryBlock", name, sizeof(GuestMemoryBlock), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestMemoryBlock_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockList(Visitor *m, GuestMemoryBlockList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestMemoryBlockList *native_i = (GuestMemoryBlockList *)i;
        visit_type_GuestMemoryBlock(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockResponseTypeList(Visitor *m, GuestMemoryBlockResponseTypeList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestMemoryBlockResponseTypeList *native_i = (GuestMemoryBlockResponseTypeList *)i;
        visit_type_GuestMemoryBlockResponseType(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockResponseType(Visitor *m, GuestMemoryBlockResponseType *obj, const char *name, Error **errp)
{
    visit_type_enum(m, (int *)obj, GuestMemoryBlockResponseType_lookup, "GuestMemoryBlockResponseType", name, errp);
}

static void visit_type_GuestMemoryBlockResponse_fields(Visitor *m, GuestMemoryBlockResponse **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_uint64(m, &(*obj)->phys_index, "phys-index", &err);
    if (err) {
        goto out;
    }
    visit_type_GuestMemoryBlockResponseType(m, &(*obj)->response, "response", &err);
    if (err) {
        goto out;
    }
    visit_optional(m, &(*obj)->has_error_code, "error-code", &err);
    if (!err && (*obj)->has_error_code) {
        visit_type_int(m, &(*obj)->error_code, "error-code", &err);
    }
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockResponse(Visitor *m, GuestMemoryBlockResponse **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestMemoryBlockResponse", name, sizeof(GuestMemoryBlockResponse), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestMemoryBlockResponse_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockResponseList(Visitor *m, GuestMemoryBlockResponseList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestMemoryBlockResponseList *native_i = (GuestMemoryBlockResponseList *)i;
        visit_type_GuestMemoryBlockResponse(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}

static void visit_type_GuestMemoryBlockInfo_fields(Visitor *m, GuestMemoryBlockInfo **obj, Error **errp)
{
    Error *err = NULL;
    visit_type_uint64(m, &(*obj)->size, "size", &err);
    if (err) {
        goto out;
    }

out:
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockInfo(Visitor *m, GuestMemoryBlockInfo **obj, const char *name, Error **errp)
{
    Error *err = NULL;

    visit_start_struct(m, (void **)obj, "GuestMemoryBlockInfo", name, sizeof(GuestMemoryBlockInfo), &err);
    if (!err) {
        if (*obj) {
            visit_type_GuestMemoryBlockInfo_fields(m, obj, errp);
        }
        visit_end_struct(m, &err);
    }
    error_propagate(errp, err);
}

void visit_type_GuestMemoryBlockInfoList(Visitor *m, GuestMemoryBlockInfoList **obj, const char *name, Error **errp)
{
    Error *err = NULL;
    GenericList *i, **prev;

    visit_start_list(m, name, &err);
    if (err) {
        goto out;
    }

    for (prev = (GenericList **)obj;
         !err && (i = visit_next_list(m, prev, &err)) != NULL;
         prev = &i) {
        GuestMemoryBlockInfoList *native_i = (GuestMemoryBlockInfoList *)i;
        visit_type_GuestMemoryBlockInfo(m, &native_i->value, NULL, &err);
    }

    error_propagate(errp, err);
    err = NULL;
    visit_end_list(m, &err);
out:
    error_propagate(errp, err);
}
